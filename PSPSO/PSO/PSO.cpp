#include <PSO/PSO.hpp>
#include <random>
#include <algorithm>
#include <cmath>


PSO::PSO(
    int subswormsAmount_, int particlesPerSubswormAmount_,
    double cognitiveCoeff_, double socialCoeff_, double pertrubationCoeff_,
    int maxOpertaions_, int dimensions_, unsigned startDynamicsFunction
) : 
    subswormsAmount(subswormsAmount_), 
    particlesPerSubswormAmount(particlesPerSubswormAmount_),
    particlesAmount(subswormsAmount * particlesPerSubswormAmount), 
    currentParticlesAmount(0), 
    currentActiveParticlesAmount(0), 
    dimensions(dimensions_),
    currentBasisFunctionsValues(std::vector<double>(dimensions)), 
    trueValue(0.),
    minActiveParticlesCoeff(0.7), 
    memoryCoeff(0.), 
    cognitiveCoeff(cognitiveCoeff_),
    socialCoeff(socialCoeff_), 
    convergenceCoeff(std::sqrt(dimensions) * 0.01), 
    optUpperBound(1.), 
    optLowerBound(-1.),
    pertrubationCoeff(pertrubationCoeff_),
    rd(), 
    randomGenerator(rd()), 
    velocityUpdateDist(0, 1),
    initializeVelocityDist(-(optUpperBound - optLowerBound)/4, +(optUpperBound - optLowerBound)/4), 
    pertrubationDist(-(optUpperBound - optLowerBound) * pertrubationCoeff, (optUpperBound - optLowerBound) * pertrubationCoeff),
    createParticleDist(optLowerBound, optUpperBound), 
    selectSubswormDist(0, subswormsAmount), 
    iterationCounter(0), operationCounter(0), 
    maxOperations(maxOpertaions_)
{
    subsworms.reserve(subswormsAmount);
    generateSubsworms(generateParticles(particlesAmount));
    // if (startDynamicsFunction >= 0 && startDynamicsFunction < dimensions)
    //     currentBasisFunctionsValues[startDynamicsFunction] = 1.;
    // else
    //     currentBasisFunctionsValues[0] = 1.;
}

// TODO: перепроверить логику второй половины
void PSO::generateSubsworms(std::vector<Particle>&& particles) {
    std::sort(
        particles.begin(), particles.end(), 
        [](const Particle& a, const Particle& b) { return a.getBestSeenFitnes() < b.getBestSeenFitnes(); }
    );

    std::vector<bool> taken(particles.size(), false);
    int taken_count = static_cast<int>(particles.size());

    while (taken_count > 0) {
        std::vector<Particle> newSubswormData;
        newSubswormData.reserve(particlesPerSubswormAmount);

        size_t i = 0;
        while (taken[i]) ++i;
        taken[i] = true;
        --taken_count;

        const auto& center = particles[i].getBestSeenPosition();

        // Находим ближайшие свободные частицы
        int needed = std::min(particlesPerSubswormAmount - 1, taken_count);
        std::vector<std::pair<double, size_t>> dists;
        dists.reserve(taken_count);
        for (size_t j = 0; j < particles.size(); ++j) {
            if (taken[j]) continue;
            double d = 0.0;
            const auto& pos = particles[j].getBestSeenPosition();
            for (int k = 0; k < dimensions; ++k) {
                double diff = pos[k] - center[k];
                d += diff * diff;
            }
            dists.emplace_back(d, j);
        }
        std::partial_sort(dists.begin(), dists.begin() + needed, dists.end());

        newSubswormData.push_back(std::move(particles[i]));
        for (int n = 0; n < needed; ++n) {
            size_t idx = dists[n].second;
            taken[idx] = true;
            --taken_count;
            newSubswormData.push_back(std::move(particles[idx]));
        }

        subsworms.emplace_back(std::move(newSubswormData), dimensions);
        currentActiveParticlesAmount += subsworms.back().getParticlesAmount();
    }
}

std::vector<Particle> PSO::generateParticles(int numberOfParticles) {
    std::vector<Particle> result;
    result.reserve(numberOfParticles);
    for (size_t i = 0; i < numberOfParticles; ++i) {
        std::vector<double> initVelocity(dimensions);
        std::vector<double> initPos(dimensions);
        for (size_t i = 0; i < dimensions; ++i) {
            initVelocity[i] = initializeVelocityDist(randomGenerator);
            initPos[i] = createParticleDist(randomGenerator);
        }
        result.emplace_back(
            initPos, initVelocity, currentBasisFunctionsValues, 
            trueValue, optLowerBound,optUpperBound
        );
    }
    return result;
}

// TODO: Перепроверить логику и математику
void PSO::indicateOverlaps() {
    bool found = true;
    while (found) {
        found = false;
        for (size_t i = 0; i < subsworms.size(); ++i) {
            if (!subsworms[i].isActive()) continue;
            for (size_t j = i + 1; j < subsworms.size(); ++j) {
                if (!subsworms[j].isActive()) continue;

                const auto& posI = subsworms[i].getBestParticle().getBestSeenPosition();
                const auto& posJ = subsworms[j].getBestParticle().getBestSeenPosition();

                double dist = 0.0;
                for (int k = 0; k < dimensions; ++k) {
                    double d = posI[k] - posJ[k];
                    dist += d * d;
                }
                dist = std::sqrt(dist);

                if (dist < subsworms[i].getInitRadius() && dist < subsworms[j].getInitRadius()) {
                    size_t worst = (subsworms[i].getBestParticle().getBestSeenFitnes() >=
                                    subsworms[j].getBestParticle().getBestSeenFitnes()) ? i : j;
                    currentActiveParticlesAmount -= subsworms[worst].getParticlesAmount();
                    subsworms.erase(subsworms.begin() + worst);
                    found = true;
                    break;
                }
            }
            if (found) break;
        }
    }
    if (static_cast<double>(currentActiveParticlesAmount) / particlesAmount < minActiveParticlesCoeff)
        performDiversityMechanism();
}

void PSO::detectSwarmsConvergence() {
    for (auto& sworm : subsworms) {
        bool isConvergent = sworm.detectConvergence(convergenceCoeff, dimensions);
        if (isConvergent) { 
            currentActiveParticlesAmount -= sworm.getParticlesAmount();
        }
    }
    if (static_cast<double>(currentActiveParticlesAmount) / particlesAmount < minActiveParticlesCoeff)
        performDiversityMechanism();
}

void PSO::makePertrubation() {
    if (subsworms.empty()) return;
    // Генерировать индекс динамически, а не из старого дистрибутора
    std::uniform_int_distribution<size_t> dist(0, subsworms.size() - 1);
    size_t subswormIndex = dist(randomGenerator);

    std::vector<std::vector<double>> pertrubations;
    pertrubations.reserve(subsworms[subswormIndex].getParticlesAmount());
    for (size_t i = 0; i < subsworms[subswormIndex].getParticlesAmount(); ++i) {
        pertrubations.emplace_back();
        pertrubations[i].reserve(dimensions);
        for (size_t j = 0; j < dimensions; ++j)
            pertrubations[i].push_back(pertrubationDist(randomGenerator));
    }
    subsworms[subswormIndex].applyPertrubationToParticles(pertrubations);
}

// TODO: Пересмотреть код
void PSO::performDiversityMechanism() {
    std::vector<std::vector<double>> savedBestPositions;
    for (auto it = subsworms.begin(); it != subsworms.end();) {
        if (!it->isActive()) {
            savedBestPositions.push_back(it->getBestParticle().getBestSeenPosition());
            it = subsworms.erase(it);
        } else {
            ++it;
        }
    }

    int numNew = particlesAmount - currentActiveParticlesAmount - static_cast<int>(savedBestPositions.size());
    std::vector<Particle> newParticles = generateParticles(numNew);

    for (const auto& pos : savedBestPositions) {
        std::vector<double> vel(dimensions);
        for (auto& v : vel) v = initializeVelocityDist(randomGenerator);
        newParticles.emplace_back(
            pos, vel, currentBasisFunctionsValues, 
            trueValue, optLowerBound, optUpperBound
        );
    }
    currentActiveParticlesAmount = 0;
    generateSubsworms(std::move(newParticles));
}

std::vector<double> PSO::iterate(const std::vector<double>& basisFunctions, double realCurrentVelocity) {
    currentBasisFunctionsValues = basisFunctions;
    trueValue = realCurrentVelocity;
    for (int iter = 0; iter < 1000; iter++) {
        for (auto& subsworm : subsworms) {
            if (subsworm.isActive()) {
                std::vector<double> randomCognitiveCoeff;
                randomCognitiveCoeff.reserve(subsworm.getParticlesAmount());
                std::vector<double> randomSocialCoeff;
                randomSocialCoeff.reserve(subsworm.getParticlesAmount());
                for (size_t i = 0; i < subsworm.getParticlesAmount(); ++i) {
                    randomCognitiveCoeff.emplace_back(velocityUpdateDist(randomGenerator));
                    randomSocialCoeff.emplace_back(velocityUpdateDist(randomGenerator));
                }
                subsworm.moveParticles(
                    memoryCoeff, cognitiveCoeff, socialCoeff, 
                    randomCognitiveCoeff, randomSocialCoeff, 
                    currentBasisFunctionsValues, trueValue
                );
            }
        }
        indicateOverlaps();
        makePertrubation();
        detectSwarmsConvergence();
    }
    // Копируем результат для возврата
    auto bestIt = std::min_element(
        subsworms.begin(), subsworms.end(),
        [](const Subsworm& a, const Subsworm& b) {
            return a.getBestParticle().getBestSeenFitnes()
                < b.getBestParticle().getBestSeenFitnes();
        }
    );
    return bestIt->getBestParticle().getBestSeenPosition();
}