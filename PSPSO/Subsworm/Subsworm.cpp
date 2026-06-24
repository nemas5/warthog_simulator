#include <Subsworm/Subsworm.hpp>
#include <cmath>

// TODO: перепроверить внутри математику
Subsworm::Subsworm(std::vector<Particle> particles_, int dimensions) : particles(std::move(particles_)), bestSwormParticleIndex(0), activity(true) {
    std::vector<double> center(dimensions, 0.0);
    for (const auto& p : particles)
        for (int k = 0; k < dimensions; ++k)
            center[k] += p.getBestSeenPosition()[k];
    for (auto& c : center) c /= particles.size();

    initRadius = 0.0;
    for (const auto& p : particles) {
        double dist = 0.0;
        const auto& pos = p.getBestSeenPosition();
        for (int k = 0; k < dimensions; ++k) {
            double d = pos[k] - center[k];
            dist += d * d;
        }
        initRadius += std::sqrt(dist);
    }
    initRadius /= particles.size();
}

void Subsworm::moveParticles(
    const double& w, const double& c1,
    const double& c2, const std::vector<double>& r1,
    const std::vector<double>& r2,
    const std::vector<double>& currentBasisFunctionsValues, const double& trueValue
) {
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].updateVelocity(w, c1, c2, r1[i], r2[i], getBestParticle().getBestSeenPosition());
        particles[i].updatePosition();
        if (particles[i].calcFitnes(currentBasisFunctionsValues, trueValue)) {
            if (particles[i].getBestSeenFitnes() < getBestParticle().getBestSeenFitnes())
                bestSwormParticleIndex = i;
        }
    }
}

// TODO: Перепроверить математику
bool Subsworm::detectConvergence(const double& convergenceCoeff, int dimensions) {
    std::vector<double> center(dimensions, 0.0);
    for (const auto& p : particles)
        for (int k = 0; k < dimensions; ++k)
            center[k] += p.getBestSeenPosition()[k];
    for (int k = 0; k < dimensions; ++k)
        center[k] /= particles.size();

    double convergence = .0;
    for (const auto& particle : particles) {
        double dist2 = 0.0;
        const auto& pos = particle.getBestSeenPosition();
        for (int k = 0; k < dimensions; ++k) {
            double diff = pos[k] - center[k];
            dist2 += diff * diff;
        }
        convergence += std::sqrt(dist2);
    }
    convergence /= particles.size();

    if (convergence < convergenceCoeff) {
        activity = false;
        return true;
    }
    return false;
}

void Subsworm::applyPertrubationToParticles(const std::vector<std::vector<double>>& pertrubations) {
    for (size_t i = 0; i < particles.size(); ++i) {
        particles[i].applyPertrubation(pertrubations[i]);
    }
}