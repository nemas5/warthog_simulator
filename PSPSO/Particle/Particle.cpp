#include <Particle/Particle.hpp>
#include <cmath>
#include <algorithm>


Particle::Particle(
    std::vector<double> initPosition, 
    std::vector<double> initVelocity,
    const std::vector<double>& currentBasisFunctionsValues,
    const double& trueValue,
    double lower, double upper
) :
    velocity(std::move(initVelocity)), 
    currentPosition(std::move(initPosition)), 
    bestSeenPosition(currentPosition),
    bestSeenFitnes(INFINITY),
    lowerBound(lower),
    upperBound(upper)
{
    calcFitnes(currentBasisFunctionsValues, trueValue);
}

bool Particle::calcFitnes(
    const std::vector<double>& currentBasisFunctionsValues, 
    const double& trueValue
) {
    double fitnes = 0;
    for (size_t i = 0; i < currentBasisFunctionsValues.size(); ++i) {
        fitnes += currentBasisFunctionsValues[i] * currentPosition[i];
    }
    fitnes -= trueValue;
    fitnes = std::abs(fitnes);
    if (fitnes < bestSeenFitnes) {
        bestSeenFitnes = fitnes;
        bestSeenPosition = currentPosition;
        return true;
    }
    return false;
}

void Particle::updateVelocity(
    const double& w, const double& c1,
    const double& c2, const double& r1,
    const double& r2, const std::vector<double>& bestSwarmPosition
) {
    for (size_t i = 0; i < velocity.size(); ++i) {
        velocity[i] = w * velocity[i]
            + c1 * r1 * (bestSeenPosition[i] - currentPosition[i])
            + c2 * r2 * (bestSwarmPosition[i] - currentPosition[i]);
    }
}

void Particle::updatePosition() {
    for (size_t i = 0; i < currentPosition.size(); ++i) {
        currentPosition[i] += velocity[i];

        if (currentPosition[i] > upperBound) {
            currentPosition[i] = upperBound;
            velocity[i] = 0.0;          // как в MATLAB
        } else if (currentPosition[i] < lowerBound) {
            currentPosition[i] = lowerBound;
            velocity[i] = 0.0;
        }
    }
}

void Particle::applyPertrubation(const std::vector<double>& pertrubation) {
    for (size_t i = 0; i < velocity.size(); ++i)
        velocity[i] += pertrubation[i];
}