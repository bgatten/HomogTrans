#include "htf/htf.h"

namespace htf{

    //helper methods
    float rad2Deg(float angle_rad){
        return 180.0/PI * angle_rad;
    }
    float deg2Rad(float angle_deg){
        return PI/180.0 * angle_deg;
    }

    Htf::Htf(){
        tf_.setIdentity();
    }

    Htf::Htf(Eigen::Matrix4d transform){
    tf_ = transform;
    }

    //todo: template this to accept different types of rotation matrices, doubles, etc.
    void Htf::setFromRotXYZ(cv::Mat rmat, double x, double y, double z){
        //Create an Eigen Matrix 
        Eigen::Matrix3d r;
        r(0,0) = rmat.at<double>(0,0);
        r(0,1) = rmat.at<double>(0,1);
        r(0,2) = rmat.at<double>(0,2);
        r(1,0) = rmat.at<double>(1,0);
        r(1,1) = rmat.at<double>(1,1);
        r(1,2) = rmat.at<double>(1,2);
        r(2,0) = rmat.at<double>(2,0);
        r(2,1) = rmat.at<double>(2,1);
        r(2,2) = rmat.at<double>(2,2);
        tf_.block<3,3>(0,0) = r;
        tf_(0,3) = x;
        tf_(1,3) = y;
        tf_(2,3) = z;
    }

    void Htf::setFromTransform(Eigen::Matrix4d Transform){
        tf_ = Transform;
    }

    void Htf::toRosCoordinateFrame(){
        double x, y, z, theta_x, theta_y, theta_z;
        decomposeTransform(x, y, z, theta_x, theta_y, theta_z);
        float x_ros = z;
        float y_ros = -x;
        float z_ros = -y;
        double roll_ros = theta_z;
        double pitch_ros = -theta_x;
        double yaw_ros = -theta_y;
        setFromRPYXYZ(roll_ros, pitch_ros, yaw_ros, x_ros, y_ros, z_ros);
    }

    void Htf::decomposeTransform(double &x, double &y, double &z, double &roll, double &pitch, double &yaw) const {
        x = tf_(0,3);
        y = tf_(1,3);
        z = tf_(2,3);
        getRPY(roll, pitch, yaw);
    }

    void Htf::getRPY(double &roll, double &pitch, double &yaw) const{
        Eigen::Matrix3d rot;
        rot = getRotation();
        double r11 = rot(0,0);
        double r21 = rot(1,0);
        double r31 = rot(2,0);
        double r32 = rot(2,1);
        double r33 = rot(2,2);
        roll = atan2(r32, r33);
        pitch = atan2(-r31, sqrt(r32*r32 + r33*r33));
        yaw = atan2(r21, r11);
    }

    Eigen::Matrix3d Htf::getRotation() const {
        Eigen::Matrix3d rot;
        rot.block<3,3>(0,0) = tf_.block<3,3>(0,0);
        return rot;
    }

    Eigen::Vector3d Htf::getTranslation() const {
        Eigen::Vector3d trans; //translation vector
        trans(0) = tf_(0, 3); //x
        trans(1) = tf_(1, 3); //y
        trans(2) = tf_(2, 3); //z
        return trans;
    }

    Eigen::Matrix4d Htf::getTransformation() const {
        return tf_;
    }

    Eigen::Quaterniond Htf::getQuaternion() const {
        // get quaternion from rotation matrix
        Eigen::Quaterniond q(this->getRotation());
        return q;
    }


    void Htf::setFromRPYXYZ(float roll, float pitch, float yaw, float x, float y, float z){
        // std::cout << "set from rpyxyz \n";
        Eigen::Matrix3d r;
        r = rot_(roll, pitch, yaw);
        tf_.block<3,3>(0,0) = r;
        // std::cout << "R " << R << std::endl;
        tf_(0,3) = x;
        tf_(1,3) = y;
        tf_(2,3) = z;
        // std::cout << "T " << Tf << std::endl;
    }

    void Htf::invert(){
        //1. Transpose the Rotation matrix
        Eigen::Matrix3d rot;
        rot = getRotation();
        rot.transposeInPlace();
        //Translation should be a 3x1
        Eigen::Vector3d p;
        p = -rot*getTranslation();
        tf_.block<3,3>(0,0) = rot;
        tf_(0,3) = p(0);
        tf_(1,3) = p(1);
        tf_(2,3) = p(2);
    }

    Htf Htf::getInverse() const {
        //1. Transpose the Rotation matrix
        Eigen::Matrix3d rot;
        rot = getRotation();
        // std::cout << "Rotation before transpose " << Rot << std::endl;
        rot.transposeInPlace();
        Eigen::Vector3d p;
        // std::cout << "Translation:\n" << getTranslation();
        // std::cout << "Rotation after transpose:\n" << Rot << std::endl;
        //Translation should be a 3x1
        p = -rot*getTranslation(); 
        // std::cout << "P\n" << P << std::endl;
        Eigen::Matrix4d transformation;
        transformation.setIdentity(); //makes 1's on the diagonal
        transformation.block<3,3>(0,0) = rot;
        transformation(0,3) = p(0);
        transformation(1,3) = p(1);
        transformation(2,3) = p(2);
        Htf htf_object;
        // htf_object.setFromRotXYZ(Rot, P(0), P(1), P(2));
        htf_object.setFromTransform(transformation);    
        return htf_object;
    }

    void Htf::setIdentity(){
        tf_.setIdentity();
    }

    Eigen::Matrix3d Htf::rot_(double roll_x, double pitch_y, double yaw_z){
        //Return an eigen matrix
        Eigen::Matrix3d rot;
        Eigen::Matrix3d rx;
        Eigen::Matrix3d ry;
        Eigen::Matrix3d rz;
        rx = rotX_(roll_x);
        ry = rotY_(pitch_y);
        rz = rotZ_(yaw_z);
        rot = rz*ry*rx;
        return rot;
    }

    Eigen::Matrix3d Htf::rotX_(double roll){
        Eigen::Matrix3d rx;
        rx << 1, 0, 0,
            0, cos(roll), -sin(roll),
            0, sin(roll), cos(roll);
        return rx;
    }
    Eigen::Matrix3d Htf::rotY_(double pitch){
        Eigen::Matrix3d ry;
        ry << cos(pitch), 0, sin(pitch),
            0, 1, 0,
            -sin(pitch), 0, cos(pitch);
        return ry;
    }

    Eigen::Matrix3d Htf::rotZ_(double yaw){
        Eigen::Matrix3d rz;
        rz << cos(yaw), -sin(yaw), 0,
            sin(yaw), cos(yaw), 0,
            0, 0, 1;
        return rz;
    }

    bool Htf::isIdentity() const {
        if (this->tf_.isIdentity(1e-5)){
            return true;
        }
        else{
            return false;
        }
    }

    void Htf::print() const {
        double x, y, z, roll, pitch, yaw;
        this->decomposeTransform(x, y, z, roll, pitch, yaw);
        std::cout << "x, y, z, r, p, y " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << std::endl;
    }

    Htf Htf::operator * (Htf &transform){
        //NOTE: These operations are NOT commutative. Order matters!
        Eigen::Matrix4d result_tf;
        result_tf = this->tf_ * transform.getTransformation(); 
        Htf result;
        result.setFromTransform(result_tf);
        return result;
    }

    Htf::~Htf(){}



}//namespace htf