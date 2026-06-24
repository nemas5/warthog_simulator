#include <Subsworm/Subsworm.hpp>
#include <Particle/Particle.hpp>
#include <vector>
#include <random>

// Алгоритм никогда не перезапускается, он просто продолжает работу, вызывая фитнес-функции, которые могут меняться.
// По сути нужно выдать коэффициенты, получить данные на новой модели, начать искать коэффициенты на новых данных.
// Настройка алгоритма - поиск подходящих мета коэффициентов и подбор такта обновления (времени на поиск).

class PSO {
    private:
        // Субпопуляции и частицы
        std::vector<Subsworm> subsworms;
        // TODO: сохранять инвариант, чтобы немного ускорить алгоритм
        // unsigned short bestCurrentSwormIndex;  // можно заменить на итератор, но надо следить тогда за инвалидацией
        void generateSubsworms(std::vector<Particle>&& particles);
        std::vector<Particle> generateParticles(int numberOfParticles);

        // Количества субпопуляций и частиц
        const int subswormsAmount;
        const int particlesPerSubswormAmount;
        const int particlesAmount;
        int currentParticlesAmount;
        int currentActiveParticlesAmount;

        // Фитнес-функция - выраженная посчитанным состоянием системы в данный момент
        int dimensions;
        std::vector<double> currentBasisFunctionsValues;
        double trueValue;

        // Мета коэффициенты алгоритма
        const double minActiveParticlesCoeff;
        const double memoryCoeff;
        const double cognitiveCoeff;
        const double socialCoeff;
        const double convergenceCoeff;
        const double optUpperBound;
        const double optLowerBound;
        double pertrubationCoeff;
        
        // Генератор и распределения для случайных значений
        std::random_device rd;
        std::mt19937 randomGenerator;
        std::uniform_real_distribution<double> velocityUpdateDist;
        std::uniform_real_distribution<double> initializeVelocityDist;
        std::uniform_real_distribution<double> pertrubationDist;
        std::uniform_real_distribution<double> createParticleDist;
        std::uniform_int_distribution<unsigned short> selectSubswormDist;

        // Статистка по итерациям и вызовам фитнес функций алгоритма
        int iterationCounter;
        int operationCounter;  // Количество операций зависит от требуемого такта обновления
        const int maxOperations;

        // Стадии итерации алгоритма
        void indicateOverlaps();
        void detectSwarmsConvergence();
        void makePertrubation();
        void performDiversityMechanism();
    public:
        PSO(
            int subswormsAmount_, int particlesPerSubswormAmount_,
            double cognitiveCoeff_, double socialCoeff_, double pertrubationCoeff_,
            int maxOpertaions_, int dimensions_, unsigned startDynamicsFunction
        );
        std::vector<double> iterate(
            const std::vector<double>& basisFunctions,
            double realCurrentVelocity
        );  // Возвращает новые коэффициенты для модели динамики

        // Для итерации нужно получить значение всех базсиных функций на этой итерации
        // И реальную скорость для этой итерации
        // Границы для значений коэффициентов можно установить в пределах 10 (?)
        // потому что трудно предположить в рассмотрении поверхность, которая будет в 100 разболее скользкой,
        // чем самая скользкая из базиса
        // Можно одновременно запустить на нескольких парамтерах
};