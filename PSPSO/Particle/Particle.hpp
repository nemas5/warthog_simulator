#pragma once
#include <vector>

class Particle {
    private:
        std::vector<double> velocity;
        std::vector<double> currentPosition;
        std::vector<double> bestSeenPosition;
        double bestSeenFitnes;
        double lowerBound;   // новые поля
        double upperBound;
    public:
        Particle(
            std::vector<double> initPosition,
            std::vector<double> initVelocity,
            const std::vector<double>& currentBasisFunctionsValues,
            const double& trueValue,
            double lower, double upper
        );
        void updatePosition();
        void updateVelocity(
            const double& w, const double& c1,
            const double& c2, const double& r1,
            const double& r2, const std::vector<double>& bestSwarmPosition
        );
        void applyPertrubation(const std::vector<double>& pertrubation);
        bool calcFitnes(const std::vector<double>& currentBasisFunctionsValues, const double& trueValue);
        const std::vector<double>& getCurrentPosition() const {return currentPosition;}
        const std::vector<double>& getBestSeenPosition() const {return bestSeenPosition;}
        const double& getBestSeenFitnes() const {return bestSeenFitnes;}
        Particle(Particle&&) noexcept = default;
        Particle& operator=(Particle&&) noexcept = default; 
};