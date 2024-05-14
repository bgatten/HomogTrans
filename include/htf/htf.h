#ifndef HTF_H
#define HTF_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <math.h>

//homogeneous transformations
namespace htf
{
    template<typename T>
    T rad2Deg(T angle_rad);
    template<typename T>
    T deg2Rad(T angle_deg);
    const double PI = 3.14159265358979323846; //const

    class Htf{
        private:
            Eigen::Matrix4d tf_; //homogeneous transformation matrix
            Eigen::Quaternionf q_; //Quaternion orientation
            Eigen::Matrix3d rot_(double roll_x, double pich_y, double yaw_z); //ZYX rotation
            Eigen::Matrix3d rotX_(double roll_x); //Rotate about x-axis
            Eigen::Matrix3d rotY_(double pitch_y); //Rotate about y-axis
            Eigen::Matrix3d rotZ_(double yaw_z); //Rotate about z-axis
        public:
            Htf(); //constructor
            Htf(Eigen::Matrix4d);
            //mutator methods
            void setFromRotXYZ(cv::Mat rmat, double x, double y, double z);
            template<typename T>
            void setFromRotXYZ(Eigen::Matrix<T, 3, 3> rmat, T x, T y, T z);
            void setIdentity();
            void setFromTransform(Eigen::Matrix4d Transform);
            void setFromRPYXYZ(double roll, double pitch, double yaw, double x, double y, double z);
            void invert();
            void toRosCoordinateFrame();
            //accessor methods
            void decomposeTransform(double &x, double &y, double &z, double &roll, double &pitch, double &yaw) const;
            void getRPY(double &roll, double &pitch, double &yaw) const;
            Htf getInverse() const;
            Eigen::Matrix3d getRotation() const;
            Eigen::Quaterniond getQuaternion() const;
            Eigen::Vector3d getTranslation() const; 
            Eigen::Matrix4d getTransformation() const;
            bool isIdentity() const;
            void print() const;
            Htf operator * (Htf &transform);
            // void getQuaternion();
            // void getRPY(); //get euler angle array
            ~Htf(); //destructor
    };

}

#endif