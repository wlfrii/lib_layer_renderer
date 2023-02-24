#include "stl_reader.h"
#include <cstdio>

namespace mlayer{

STLReader::STLReader()
    : _is_stl_exist(false)
{
}


STLReader* STLReader::getInstance()
{
    static STLReader reader;
    return &reader;
}


/**
 * @brief STL data.
 *
 * The structure of binary STL file is as follows.
 *
 * + [80] There is a "80-bytes file header" storing the name of the file
 * + [4]  Followed by a 4-bytes integer number describ the number of triangle
 *        patch
 * + [50*n] A triangle patch use fixed 50-bytes, including
 *       +  [12] Three 4-bytes float number store the normal
 *       +  [12] Three 4-bytes float number store first vertex
 *       +  [12] Three 4-bytes float number store second vertex
 *       +  [12] Three 4-bytes float number store third vertex
 *       +  [2]  The last 2-bytes used to describ properties
 */

bool STLReader::read(const std::string &filename, Vertices &data)
{
    if(!_is_stl_exist){
        _rot = glm::mat3(1.0f);
        _t = glm::vec3(0.0f);
    }

    std::FILE* fp = fopen(filename.c_str(), "rb");
    if(fp == nullptr){
        throw "Failed to read STL file.\n";
        return false;
    }

    char name[80];
    int face_num = 0;
    fread(&name, 80, 1, fp);
    fread(&face_num, 4, 1, fp);
    //MTRACE("Read STL file: '%s', [%d] faces.\n", name, face_num);

    data.clear();
    data.resize(face_num * 3);
    for(int i = 0; i < face_num; ++i){
        glm::vec3 n, p1, p2, p3;
        char info[2];

        fread(&n, 12, 1, fp);
		fread(&p1, 12, 1, fp);
		fread(&p2, 12, 1, fp);
		fread(&p3, 12, 1, fp);
		fread(&info,2, 1, fp);

        // For the existing model, transform the vertices
        //n = _rot * n;
        p1 = _rot * p1 + _t;
        p2 = _rot * p2 + _t;
        p3 = _rot * p3 + _t;

        data[3*i + 0] = {glm::vec4(p1, 1.f), glm::vec4(n, 1.f)};
        data[3*i + 1] = {glm::vec4(p2, 1.f), glm::vec4(n, 1.f)};
        data[3*i + 2] = {glm::vec4(p3, 1.f), glm::vec4(n, 1.f)};
    }
    fclose(fp);

    return true;
}


bool STLReader::read(STLModelType type, Vertices &data)
{
    _rot = glm::mat3(1.0f);
    _t = glm::vec3(0.0f);
    switch (type)
    {
    case STL_NH_0_SIMPLIFIED:
    {
        // Assign along colunms
        _rot = glm::mat3(1, 0, 0, 0, cos(M_PI), sin(M_PI), 0, -sin(M_PI), cos(M_PI));
        _t = glm::vec3(-2.636925f, 3.f, 14.68f);
        _is_stl_exist = true;
        return read("./models/nh_0_simplified.STL", data);
    }
    case STL_NH_1_SIMPLIFIED:
    {
        _rot = glm::mat3(1, 0, 0, 0, cos(-M_PI/2.f), sin(-M_PI/2.f), 0, -sin(-M_PI/2.f), cos(-M_PI/2.f));
        _t = glm::vec3(-1.8f, -2.534f, 14.68f);
        _is_stl_exist = true;
        return read("./models/nh_1_simplified.STL", data);
    }
    case STL_NH_0:
    {
        // Assign along colunms
        _rot = glm::mat3(1, 0, 0, 0, cos(M_PI), sin(M_PI), 0, -sin(M_PI), cos(M_PI));
        _t = glm::vec3(-2.637f, 3.f, 15.08f);
        _is_stl_exist = true;
        return read("./models/nh_0.STL", data);
    }
    case STL_NH_1:
    {
        // Assign along colunms
        _rot = glm::mat3(1, 0, 0, 0, cos(-M_PI/2.f), sin(-M_PI/2.f), 0, -sin(-M_PI/2.f), cos(-M_PI/2.f));
        _t = glm::vec3(-1.8f, -2.427f, 15.08f);
        _is_stl_exist = true;
        return read("./models/nh_1.STL", data);
    }
    case STL_BGF_0:
    {
        // Assign along colunms
        _rot = glm::mat3(cos(M_PI), 0, sin(M_PI), 0, 1, 0, -sin(M_PI), 0, cos(M_PI));
        _t = glm::vec3(3.467f, -3.337f, 26.65f);
        _is_stl_exist = true;
        return read("./models/bgf_0.STL", data);
    }
    case STL_BGF_1:
    {
        // Assign along colunms
        _rot = glm::mat3(cos(M_PI), 0, sin(M_PI), 0, 1, 0, -sin(M_PI), 0, cos(M_PI));
        _t = glm::vec3(0.973f, -2.2782f, 26.65f);
        _is_stl_exist = true;
        return read("./models/bgf_1.STL", data);
    }
    case STL_TGF_0:
    {
        // Assign along colunms
        _rot = glm::mat3(cos(-M_PI/2.f), 0, -sin(-M_PI/2.f), 0, 1, 0, sin(-M_PI/2.f), 0, cos(-M_PI/2.f));
        _t = glm::vec3(3.283f, -3.287f, -3.f);
        _is_stl_exist = true;
        return read("./models/tgf_0.STL", data);
    }
    case STL_TGF_1:
    {
        // Assign along colunms
        _rot = glm::mat3(cos(-M_PI/2.f), 0, -sin(-M_PI/2.f), 0, 1, 0, sin(-M_PI/2.f), 0, cos(-M_PI/2.f));
        _t = glm::vec3(1.6275f, -2.7078f, -2.9957f);
        _is_stl_exist = true;
        return read("./models/tgf_1.STL", data);
    }
    case STL_TGF_2:
    {
        // Assign along colunms
        _rot = glm::mat3(cos(-M_PI/2.f), 0, -sin(-M_PI/2.f), 0, 1, 0, sin(-M_PI/2.f), 0, cos(-M_PI/2.f));
        _t = glm::vec3(3.0972f, -2.7078f, -2.997f);
        _is_stl_exist = true;
        return read("./models/tgf_2.STL", data);
    }
    default:
        _is_stl_exist = false;
        break;
    }
    return false;
}

} // namespace::mlayer