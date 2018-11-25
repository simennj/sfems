#ifndef SFEMS_ITERATOR_HPP
#define SFEMS_ITERATOR_HPP


class Iterator {
protected:
    int maxIterations;
    int currentIteration = 0;
public:
    explicit Iterator(int maxIterations) : maxIterations(maxIterations) {}
    virtual bool iterate(double residual) = 0;
    void resetIterations();
    bool reachedMaxIterations();
};

class SimpleIterator : public Iterator {
public:
    explicit SimpleIterator(int maxIterations) : Iterator(maxIterations) {}

private:
    bool iterate(double residual) override;
};

class NewtonIterator : public Iterator {
    double tolerance;
public:
    explicit NewtonIterator(int maxIterations, double tolerance) : Iterator(maxIterations), tolerance(tolerance) {}

private:
    bool iterate(double residual) override;
};

#endif //SFEMS_ITERATOR_HPP
