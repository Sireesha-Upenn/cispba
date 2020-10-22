#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>
#include <string>
#include <fstream>

template <class T, int dim>
class MassSpringSystem
{
public:
    using TV = Eigen::Matrix<T, dim, 1>;

    std::vector<Eigen::Matrix<int, 2, 1>> segments;
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    T youngs_modulus;
    T damping_coeff;
    std::vector<bool> node_is_fixed;
    std::vector<T> rest_length;

    MassSpringSystem()
    {
    }

    void evaluateSpringForces(std::vector<TV> &f)
    {
        // TODO: evaluate spring force
        //Initialize and resize f to be the size of x
        f.clear();
        f.resize(x.size(), TV::Zero());

        for (size_t i = 0; i < segments.size(); i++)
        {
            int p1 = segments[i][0], p2 = segments[i][1];
            T length = (x[p1] - x[p2]).norm();
             //Unit normal direction 
             //n12 = x1 - x2 / |x1 - x2|
            TV n12 = (x[p1] - x[p2]).normalized();
            TV force = (-youngs_modulus) * (length / (rest_length[i] - T(1))) * n12;
            //f1 is -f2 due to conservation of momentum 
            f[p1] += force; f[p2] -= force;
        }
    }

    void evaluateDampingForces(std::vector<TV> &f)
    {
        // TODO: evaluate damping force
        f.clear();
        f.resize(x.size(), TV::Zero());

        for (size_t i = 0; i < segments.size(); i++)
        {
            uint p1 = segments[i][0], p2 = segments[i][1];
            //Unit normal direction 
            //n12 = x1 - x2 / |x1 - x2|
            TV n = (x[p1] - x[p2]).normalized();
            Eigen::Matrix<T, dim, dim> nnt = n * n.transpose(); 
            TV force = -damping_coeff * nnt * (v[p1] - v[p2]); 
             //f1 is -f2 due to conservation of momentum 
            f[p1] += force; f[p2] -= force;
        }
    }

    void dumpPoly(std::string filename)
    {
        std::ofstream fs;
        fs.open(filename);
        fs << "POINTS\n";
        int count = 0;
        for (auto X : x)
        {
            fs << ++count << ":";
            for (int i = 0; i < dim; i++)
                fs << " " << X(i);
            if (dim == 2)
                fs << " 0";
            fs << "\n";
        }
        fs << "POLYS\n";
        count = 0;
        for (const Eigen::Matrix<int, 2, 1> &seg : segments)
            fs << ++count << ": " << seg(0) + 1 << " " << seg(1) + 1 << "\n"; // poly segment mesh is 1-indexed
        fs << "END\n";
        fs.close();
    }
};
