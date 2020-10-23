#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <random>
#include <chrono>
#include <unordered_set>
#include <unordered_map>
#include <bits/stdc++.h>
#include <map>
#include <utility>

#include "SimulationDriver.h"

//#define STRUCT_SPRINGS
//#define SHEARING_SPRINGS
#define BENDING_SPRINGS

using namespace std;

int main(int argc, char *argv[])
{
    using T = float;
    constexpr int dim = 3;
    using TV = Eigen::Matrix<T, dim, 1>;

    SimulationDriver<T, dim> driver;

    // set up mass spring system
    T youngs_modulus = 0;
    T damping_coeff = 0;
    T dt = 0;

    // node data
    std::vector<T> m;
    std::vector<TV> x;
    std::vector<TV> v;
    std::vector<bool> node_is_fixed; //initialize to false after init x //initialize length of vector based on size of x
    //assign border conditions here

    // segment data
    std::vector<Eigen::Matrix<int, 2, 1>> segments;
    std::vector<T> rest_length;

    if (argc < 2)
    {
        std::cout << "Please indicate test case number: 0 (cloth) or 1 (volumetric bunny)" << std::endl;
        exit(0);
    }

    //WORKS!!
    if (strcmp(argv[1], "0") == 0) // cloth case
    {
        //Resize the vectors
        int total_points = 64 * 64;
        x.resize(total_points);
        v.resize(total_points);
        m.resize(total_points);
        int n = 64;
        T dx = T(1) / (n - 1);

        // TODO
        // 1. Create node data: position, mass, velocity
        float total_mass = 5;
        //Fill in positions
        for (int i = 0; i < n; ++i) //Build on XZ axis
        {
            for (int k = 0; k < n; k++)
            {
                //Fill in x and z positions
                int index = (n * i) + k;
                //Pos
                x[index][0] = (i - 1) * dx;
                x[index][1] = T(1);
                x[index][2] = (k - 1) * dx;
                //Vel
                v[index] = TV::Zero(); //zero vector
                //Mass
                m[index] = T(total_mass / total_points);
            }
        }
        //initialize node_is_fixed to the size of x
        node_is_fixed.resize(total_points, false);

        for(int i = 0; i < 64; ++i)
        {
            for(int j = 0; j < 64; ++j)
            {
                cout<<i<<" 0 "<<j<<std::endl; 
                cout<<i<<" 0 "<<j + 1<<std::endl; 
                cout<<i + 1<<" 0 "<<j<<std::endl; 
                cout<<i + 1<<" 0 "<<j + 1<<std::endl; 
            }
        }

        //2.
        //Fill segments and rest_length, including struct springs, shearing springs and bending springs.
        //Segments and rest length
        //Store indices in a matrix for better access to neighboring nodes
#ifdef STRUCT_SPRINGS
        //Vertical segments
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i + 1) * n + j;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Horizontal Segments
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
#endif
#ifdef SHEARING_SPRINGS
        //Vertical segments
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i + 1) * n + j;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Horizontal Segments
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Diagonal segments top left to bottom right
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = ((i + 1) * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Diagonal Segments bottom left to top right
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = ((i + 1) * n) + j;
                int p2 = (i * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }

#endif
#ifdef BENDING_SPRINGS
        //Vertical segments
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i + 1) * n + j;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Horizontal Segments
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < (n - 1); ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Diagonal segments top left to bottom right
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < (n - 1); ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = ((i + 1) * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Diagonal Segments bottom left to top right
        for (int i = 0; i < (n - 1); ++i)
        {
            for (int j = 0; j < (n - 1); ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = ((i + 1) * n) + j;
                int p2 = (i * n) + j + 1;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Alternate Vertical segments
        for (int i = 0; i < (n - 2); ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i + 2) * n + j;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }
        //Alternate Horizontal Segments
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < (n - 2); ++j)
            {
                Eigen::Matrix<int, 2, 1> curr;
                int p1 = (i * n) + j;
                int p2 = (i * n) + j + 2;
                curr << p1, p2;
                segments.push_back(curr);
                T len = (x[p1] - x[p2]).norm();
                rest_length.push_back(len);
            }
        }

#endif

        //OKOKOKOKOKOKOK
        //3.
        //Choose proper youngs_modulus, damping_coeff, dt;
        youngs_modulus = T(1.5);
        damping_coeff = T(3.3);
        dt = T(0.00001);//further decrease it 

        //4.
        //Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
        node_is_fixed[0] = true;
        node_is_fixed[64] = true;

        //Check if values are ok by printing them
        //std::cout<<"Segment values are "<<std::endl;
        // for(size_t i = 0; i < segments.size(); i++)
        // {
        //     std::cout<<"( "<<segments[i][0]<<" "<<segments[i][1]<<" )"<<std::endl;
        // }
        // for(size_t i = 0; i < m.size(); i++)
        // {
        //     std::cout<<"( "<<m[i]<<" )"<<std::endl;
        // }

        //todo
        driver.helper = [&](T t, T dt) {
            //move the positions for fixed nodes
        };
        driver.test = "cloth";
    }

    else if (strcmp(argv[1], "1") == 0) // volumetric bunny case
    {
        //1. Create node data from data/points: The first line indicates the number of points and dimension (which is 3).
        //Position data from file
        //Fill out velocity and mass
        //-------------
        //DATA/POINTS
        //-------------
        float total_mass = 20;
        string filename = "data/pointstxt.txt";
        std::ifstream datafile(filename.c_str(), std::ios_base::in);
        //datafile.open("", std::ios_base::in);
        if (!datafile)
        {
            std::cerr << "Unable to open data file" << std::endl;
        }
        int num_points = 0, dimension = 0;
        //First line represents number of points and dimension of the data
        datafile >> num_points >> dimension;

        std::cout << "Num points: " << num_points << std::endl;
        std::cout << "Dimension: " << dimension << std::endl;

        x.resize(num_points);
        v.resize(num_points);
        m.resize(num_points);

        std::vector<T> tempvals;
        while (!datafile.eof())
        {
            T val;
            datafile >> val;
            tempvals.push_back(val);
        }
        for (int count = 0; count < num_points; count++)
        {
            for (int dim = 0; dim < dimension; dim++)
            {
                x[count][dim] = tempvals[(count * dimension) + dim];
                //std::cout<<"X val entered was" <<x[count][dim]<<std::endl;
                //std::cout<<"count is"<<count<<std::endl;
                v[count][dim] = T(0);
            }
            m[count] = total_mass / num_points; //Fill mass
        }

        datafile.close();
        // //initialize node_is_fixed bool vector to the size of x
        node_is_fixed.resize(num_points, false);

        //2. Fill segments and rest_length from data/cells: The first line indicates the number of tetrahedra and the number of vertices of each tet (which is 6).
        //Each edge in this tetrahedral mesh will be a segment. Be careful not to create duplicate edges.

        //-------------
        //DATA/CELLS
        //-------------

        //Open the cells file
        string filename2 = "data/cellstxt.txt";
        std::ifstream cellsfile(filename2.c_str(), std::ios_base::in);
        //datafile.open("", std::ios_base::in);
        if (!datafile)
        {
            std::cerr << "Unable to open data file" << std::endl;
        }
        int num_tetrahedrons = 0, tetra_dimension = 0;

        //First line represents number of points and dimension of the data
        cellsfile >> num_tetrahedrons >> tetra_dimension;

        std::cout << "Num tetrahedrons: " << num_tetrahedrons << std::endl;
        std::cout << "tetra dim: " << tetra_dimension << std::endl;

        //Rest of the lines
        //Reading cells file data into input and pushing it back to celldata vector
        std::vector<int> indices;
        //indices.resize(num_tetrahedrons * tetra_dimension);

        while (!cellsfile.eof())
        {
            int val;
            cellsfile >> val;
            indices.push_back(val);
        }
        cellsfile.close(); //Close the file

        //ALL GOOD UP TO HERE

        //Fill in edges here from the data read from cells file
        //Each tetrahedron will have 6 edges
        //Check for duplicate edges
        //Calculate segments from tetrahedrons
        std::unordered_multimap<int, int> alledges;
        for (int count = 0; count < num_tetrahedrons; count += tetra_dimension)
        {
            alledges.insert(std::make_pair(indices[count], indices[count + 1]));
            alledges.insert(std::make_pair(indices[count], indices[count + 2]));
            alledges.insert(std::make_pair(indices[count], indices[count + 3]));
            alledges.insert(std::make_pair(indices[count + 1], indices[count + 2]));
            alledges.insert(std::make_pair(indices[count + 2], indices[count + 3]));
            alledges.insert(std::make_pair(indices[count + 1], indices[count + 3]));
        }

        //Get rid of duplicate edges
        for (auto edge : alledges)
        {
            auto second = edge.second;
            if (alledges.find(second) != alledges.end())
            {
                auto found = alledges.find(second);
                if (edge.first == found->second)
                {
                    alledges.erase(edge.first);
                }
            }
        }
        //Resize segments
        segments.resize(alledges.size());
        int seg_count = 0;
        for (auto edge : alledges)
        {
            segments[seg_count][0] = edge.first;
            segments[seg_count][1] = edge.second;
            seg_count++;
        }
        //Fill in rest_length
        rest_length.resize(segments.size());
        for (size_t i = 0; i < segments.size(); i++)
        {
            int p1 = segments[i][0];
            int p2 = segments[i][1];
            T len = (x[p1] - x[p2]).norm();
            rest_length.push_back(len);
        }

        //Check if values are ok by printing them
        // std::cout << "Segment values are " << std::endl;
        // for (size_t i = 0; i < segments.size(); i++)
        // {
        //     std::cout << "( " << segments[i][0] << " " << segments[i][1] << " )" << std::endl;
        // }

        //3.
        //Choose proper youngs_modulus, damping_coeff, dt;
        youngs_modulus = T(1.5);
        damping_coeff = T(3.3);
        dt = T(0.00001);//further decrease it 

        // //4.
        // //Set boundary condition (node_is_fixed) and helper function (to achieve moving boundary condition).
        node_is_fixed[2140] = true; //create a vector of length same as x
        node_is_fixed[2346] = true;
        node_is_fixed[1036] = true;

        // TODO
        //called during simulation
        driver.helper = [&](T t, T dt) {
            //modify the data time t
            //modify node_is_fixed
            //move the fixed fixed nodes in time
            if(t > 4)
            {
                driver.ms.node_is_fixed[1036] = false; //release tail node 
                node_is_fixed[1036] = false;
                //x[idx]= (, , ); 
                //driver.ms.x & v
                //release tail at some time
                //modify the node_isfixed for diver.ms 
            }
        };
        driver.test = "bunny";
    }

    else
    {
        std::cout << "Wrong case number!" << std::endl;
        exit(0);
    }

    // simulate

    driver.dt = dt;
    driver.ms.segments = segments;
    driver.ms.m = m;
    driver.ms.v = v;
    driver.ms.x = x;
    driver.ms.youngs_modulus = youngs_modulus;
    driver.ms.damping_coeff = damping_coeff;
    driver.ms.node_is_fixed = node_is_fixed;
    driver.ms.rest_length = rest_length;

    driver.run(120);

    return 0;
}
