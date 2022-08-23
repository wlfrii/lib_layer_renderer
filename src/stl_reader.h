#ifndef STL_READER_H_LF
#define STL_READER_H_LF
#include <string>
#include <vector>
#include <glm/glm.hpp>
#include "layer_define.h"

/**
 * @brief The existing STL model. 
 */
enum STLModelType
{
    STL_NH_0_SIMPLIFIED,
    STL_NH_1_SIMPLIFIED,
    STL_NH_0,
    STL_NH_1,
    STL_BGF_0,
    STL_BGF_1,
    STL_TGF_0,
    STL_TGF_1,
    STL_TGF_2,
};


/**
 * @brief STL model reader 
 */
class STLReader
{
protected:
    STLReader();

public:
    ~STLReader() {}


    /**
     * @brief Create and return a single instance.
     * @return
     */
    static STLReader* getInstance();


    /**
     * @brief Read binary STL model specified by filename
     * 
     * @param filename  The filename of STL file
     * @param data  The STL data of the model
     * @return 
     *  - true, if succeed to read 
     *  - false, otherwise 
     */
    bool read(const std::string &filename,  Vertices& data);
 

    /**
     * @brief Read the existing binary STL model
     * 
     * @param type  The existing STL type
     * @param data  The STL data of the STL model
     * @return 
     *  - true, if succeed to read 
     *  - false, otherwise 
     */
    bool read(STLModelType type, Vertices &data);

private:    
    glm::mat3 _rot;     // The pre-rotation for STL vertices
    glm::vec3 _t;       // The pre-tranlation for STL vertices

    bool _is_stl_exist; // The flag for existing STL model
};

#endif // STL_READER_H_LF
