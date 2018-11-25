#include <GLFW/glfw3.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "graphics/graphics.hpp"
#include "utils/arch.hpp"
#include "calculations/linear.hpp"
#include "utils/fileUtils.hpp"
#include "utils/logFunctions.hpp"
#include "calculations/BeamElement.hpp"
#include "calculations/corotational.hpp"
#include "calculations/iterator.hpp"

extern "C" {
    #include <time.h>
    #include "graphics/window_util.h"
}

double initialViewWidth;
double viewWidth;
double increments;
std::unique_ptr<Analyzer> analyzer; // TODO: find a cleaner way to handle analyzer state?

std::unique_ptr<Iterator> createIterator(YAML::Node config) {
    int maxIterationsPerIncrement = config["maxIterationsPerIncrement"].as<int>();
    if (config["type"].as<std::string>() == "newton") {
        return std::make_unique<NewtonIterator>(maxIterationsPerIncrement, config["tolerance"].as<double>());
    } else {
        return std::make_unique<SimpleIterator>(maxIterationsPerIncrement);
    }
}

std::unique_ptr<Analyzer> loadStuff() {
    YAML::Node config = YAML::LoadFile("config.yaml");

    auto curveConfig = config["curve"];
    CurveExpression* expression;
    if (curveConfig["type"].as<std::string>() == "arch"){
        expression = new CircleExpression(
                curveConfig["radius"].as<double>(),
                curveConfig["height"].as<double>());
    } else {
        expression = new LineExpression(
                curveConfig["length"].as<double>(),
                curveConfig["height"].as<double>());
    }
    const auto elementCount = config["elementCount"].as<unsigned int>();
    auto vertices = calculateArch(elementCount, expression);
    delete expression;
    auto properties = ElementProperties{
                        config["youngsModulus"].as<double>(),
                        config["crossSectionArea"].as<double>(),
                        config["momentOfIntertia"].as<double>()
                };
    std::vector<BoundaryCondition> boundaryConditions{};
    for (auto boundaryCondition : config["boundaryConditions"]) {
        boundaryConditions.push_back(BoundaryCondition{
            boundaryCondition["globalDegreeOfFreedom"].as<int>(),
            boundaryCondition["value"].as<double>()
        });
    }

    std::vector<Force> forceVector{};
    for (auto forceNode : config["force"]) {
        int node;
        if (forceNode["node"].as<std::string>() == "middle") {
            node = elementCount/2;
        } else {
            node = forceNode["node"].as<int>();
        }
        ForceType forceType;
        if (forceNode["type"].as<std::string>() == "local") {
            forceType = ForceType::LOCAL;
        } else {
            forceType = ForceType::GLOBAL;
        }
        forceVector.push_back(Force{
                forceType,
                node,
                forceNode["degreeOfFreedom"].as<int>(),
                forceNode["magnitude"].as<double>()
        });
    }

    viewWidth = initialViewWidth = config["viewWidth"].as<double>();
    auto iteratorConfig = config["iterator"];
    increments = iteratorConfig["increments"].as<int>();
    std::unique_ptr<Iterator> iterator = createIterator(iteratorConfig);

    if (config["analyzer"].as<std::string>() == "corotational") {
        return std::make_unique<Corotational>(vertices, properties, boundaryConditions, forceVector, std::move(iterator));
    } else {
        return std::make_unique<Linear>(vertices, properties, boundaryConditions, forceVector, std::move(iterator));
    }

//    logVertices(analyzer.getVertices());
}

/*
 * Handles all user input, is called from window_utils.c
 */
void key_callback(GLFWwindow *window, int key, int scancode, int action, int mode) {
    if (action == GLFW_PRESS) {
        switch (key) {
            case GLFW_KEY_ESCAPE:
                // Closes the window which quits the program
                glfwSetWindowShouldClose(window, GL_TRUE);
                break;
            case GLFW_KEY_R:
                analyzer = loadStuff();
                // Reloads shaders and ui points from shader, vertices and indices files
                graphics_reload();
                // Reinitializes PID controller with values from constants file
                break;
            case GLFW_KEY_DOWN:
                analyzer->setForceScale(analyzer->getForceScale()+.05);
//                std::cout << "localForces: " << analyzer->getLocalForces().transpose()*analyzer->getForceScale() << std::endl;
//                std::cout << "globalForces: " << analyzer->getGlobalForces().transpose()*analyzer->getForceScale()  << std::endl;
                break;
            case GLFW_KEY_UP:
                analyzer->setForceScale(analyzer->getForceScale()-.05);
//                std::cout << "localForces: " << analyzer->getLocalForces().transpose()*analyzer->getForceScale()  << std::endl;
//                std::cout << "globalForces: " << analyzer->getGlobalForces().transpose()*analyzer->getForceScale()  << std::endl;
                break;
            case GLFW_KEY_SPACE:
                analyzer->iterate(increments);//1.0/increments);
//                analyzer->setForceScale(analyzer->getForceScale()+1);
//                std::cout << "localForces: " << analyzer->getLocalForces().transpose()*analyzer->getForceScale()  << std::endl;
//                std::cout << "globalForces: " << analyzer->getGlobalForces().transpose()*analyzer->getForceScale()  << std::endl;
//                analyzer->update(0.01, force);
//                force += initialForce;
                break;
            case GLFW_KEY_0: {
                std::vector<double> vertices = analyzer->getVertices();
                std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                               std::abs<double>);

                viewWidth = initialViewWidth =
                        *std::max_element(vertices.begin(), vertices.end())*2;
                break;
            }
            case GLFW_KEY_RIGHT:
                analyzer->update();
                break;
            case GLFW_KEY_I: {
                //iterate
                for (int i = 0; i < increments; ++i) {
                    const auto diverging = analyzer->iterate(increments);//1.0 / increments);
                    if (diverging) break;
                }
                break;
            }
            case GLFW_KEY_J: {
                //iterate
                for (int i = 0; i < 100; ++i) {
                    const auto diverging = analyzer->iterate(increments);//1.0 / increments);
                    if (diverging) break;
                }
                break;
            }
            case GLFW_KEY_MINUS:
                viewWidth += initialViewWidth/20;
                break;
            case GLFW_KEY_EQUAL:
                viewWidth -= initialViewWidth/20;
            default:
                break;
        }
    }
}

/*
 * The main function
 */
int main(int argc, char **argv) {
    // Initializes main modules
    window_init(key_callback);
    graphics_init((void *(*)(const char)) glfwGetProcAddress);

    analyzer = loadStuff();
//    analyzer->update();

    while (window_open()) {
        // Handle input and draw updated values
        window_update_wait();
        std::vector<double> rawVertices{analyzer->getVertices()};
        std::vector<float> vertices(rawVertices.size());
        std::transform (rawVertices.begin(), rawVertices.end(), vertices.begin(), [](double v){ return 2*v/viewWidth;});
        graphics_draw(vertices);
    }
    return 0;
}
