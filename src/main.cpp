#include <GLFW/glfw3.h>
#include <iostream>
#include <yaml-cpp/yaml.h>
#include "graphics/graphics.hpp"
#include "utils/arch.hpp"
#include "utils/fileUtils.hpp"
#include "calculations/BeamElement.hpp"
#include "calculations/logger.hpp"
#include "calculations/structure.hpp"

extern "C" {
#include "graphics/window_util.h"
}

double initialViewWidth;
double viewWidth;
double increments;
std::unique_ptr<Structure> structure;

int maxIterations;
double tolerance;

bool arclength;

double stepSize;

std::unique_ptr<Structure> loadStuff() {
    YAML::Node config = YAML::LoadFile("config.yaml");

    auto curveConfig = config["curve"];
    CurveExpression *expression;
    if (curveConfig["type"].as<std::string>() == "arch") {
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
    int degreesOfFreedom = (elementCount + 1) * 3;
    std::vector<BoundaryCondition> boundaryConditions{};
    for (auto boundaryCondition : config["boundaryConditions"]) {
        auto degreeOfFreedom = boundaryCondition["globalDegreeOfFreedom"].as<int>();
        boundaryConditions.push_back(BoundaryCondition{
                degreeOfFreedom < 0 ? degreesOfFreedom + degreeOfFreedom : degreeOfFreedom,
                boundaryCondition["value"].as<double>()
        });
    }

    std::vector<Force> forceVector{};
    for (auto forceNode : config["force"]) {
        int node;
        if (forceNode["node"].as<std::string>() == "middle") {
            node = elementCount / 2;
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

    unsigned int nodeToLog;
    if (config["logging"]["node"].as<std::string>() == "middle")
        nodeToLog = elementCount / 2;
    else
        nodeToLog = config["logging"]["node"].as<unsigned int>();
    Logger logger = Logger(nodeToLog, config["logging"]["degreeOfFreedom"].as<unsigned int>());

    viewWidth = initialViewWidth = config["viewWidth"].as<double>();
    auto iteratorConfig = config["iterator"];
    increments = iteratorConfig["increments"].as<int>();
    maxIterations = iteratorConfig["maxIterationsPerIncrement"].as<int>();
    tolerance = iteratorConfig["tolerance"].as<double>();
    arclength = !(iteratorConfig["type"].as<std::string>() == "newton");
    if (iteratorConfig["stepSize"].IsDefined())
        stepSize = iteratorConfig["stepSize"].as<double>();
    else
        stepSize = 1.0 / increments;

    return std::make_unique<Structure>(vertices, properties, boundaryConditions, forceVector, std::move(logger));
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
                structure = loadStuff();
                // Reloads shaders and ui points from shader, vertices and indices files
                graphics_reload();
                break;
            case GLFW_KEY_SPACE:
                if (arclength)
                    structure->arcLength(stepSize, tolerance, maxIterations);
                else
                    structure->newton(stepSize, tolerance, maxIterations);
                break;
            case GLFW_KEY_0: {
                std::vector<double> vertices = structure->getVertices();
                std::transform(vertices.begin(), vertices.end(), vertices.begin(),
                               std::abs<double>);

                viewWidth = initialViewWidth =
                        *std::max_element(vertices.begin(), vertices.end()) * 2;
                break;
            }
            case GLFW_KEY_I: {
                //iterate
                for (int i = 0; i < increments; ++i) {
                    bool diverging;
                    if (arclength)
                        diverging = structure->arcLength(stepSize, tolerance, maxIterations);
                    else
                        diverging = structure->newton(stepSize, tolerance, maxIterations);
                    if (diverging) break;
                }
                break;
            }
            case GLFW_KEY_MINUS:
                viewWidth += initialViewWidth / 20;
                break;
            case GLFW_KEY_EQUAL:
                viewWidth -= initialViewWidth / 20;
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

    structure = loadStuff();

    while (window_open()) {
        // Handle input and draw updated values
        window_update_wait();
        std::vector<double> rawVertices{structure->getVertices()};
        std::vector<float> vertices(rawVertices.size());
        std::transform(rawVertices.begin(), rawVertices.end(), vertices.begin(),
                       [](double v) { return 2 * v / viewWidth; });
        graphics_draw(vertices);
    }
    return 0;
}
