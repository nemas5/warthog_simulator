#include <Particle/Particle.hpp>
#include <vector>

class Subsworm {
    private:
        std::vector<Particle> particles;
        unsigned bestSwormParticleIndex;
        bool activity;
        double initRadius;
    public:
        Subsworm(std::vector<Particle>, int dimensions);
        void moveParticles(
            const double& w, const double& c1,
            const double& c2, const std::vector<double>& r1,
            const std::vector<double>& r2,
            const std::vector<double>& currentBasisFunctionsValues, const double& trueValue
        );
        bool detectConvergence(const double& convergenceCoeff, int dimensions);
        void applyPertrubationToParticles(const std::vector<std::vector<double>>& pertrubations);
        const Particle& getBestParticle() const {return particles[bestSwormParticleIndex];};
        const unsigned getParticlesAmount() const {return particles.size();};
        const bool& isActive() const {return activity;};
        const double& getInitRadius() const {return initRadius;}
};