#include "circle_generator.h"
#include "util_generator.h"

CircleGenerator::CircleGenerator()
    : _points_num(0)
    , _radius(0)
{
    _vertices.clear();
}


CircleGenerator::CircleGenerator(int points_num, float radius,
                                 const glm::vec3& origin)
    : _points_num(points_num)
    , _radius(radius)
    , _origin(origin)
{
    assert(points_num >= 3);

    // Build circle points
    _positions.resize(points_num);
    double angle = 2.0*M_PI / _points_num;
    for(int i = 0; i < _points_num; i++){
        float delta = angle * i;
        float x = _origin.x + radius * glm::cos(delta);
        float y = _origin.y + radius * glm::sin(delta);
        float z = _origin.z;
        _positions[i] = glm::vec4(x, y, z, 1);
    }

    // Build circle faces
    _vertices.resize(_points_num * 3);
    glm::vec4 p = glm::vec4(_origin, 1.f);
    for(int i = 0; i < _points_num; i++){
        glm::vec4 p1 = _positions[i];
        glm::vec4 p2 = _positions[(i + 1) % _points_num];
        glm::vec4 n = getNormal(p1, p, p2);

        _vertices.push_back({p1, n});
        _vertices.push_back({p2, n});
        _vertices.push_back({p, n});
    }
}


const Vertices &CircleGenerator::result() const
{
    return _vertices;
}


const Positions& CircleGenerator::positions() const
{
    return _positions;
}
