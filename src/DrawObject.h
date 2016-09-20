//
// Created by juarez on 19/09/16.
//

#ifndef BLENDSHAPESVIEWER_DRAWOBJECT_H
#define BLENDSHAPESVIEWER_DRAWOBJECT_H

#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class DrawObject {
public:
    GLuint vb;  // vertex buffer
    int numTriangles;
    std::vector<float> attributes;  // pos(3float), normal(3float), color(3float)

    DrawObject() { }

    DrawObject(GLuint vb, int t, std::vector<float> attr) {
        this->vb = vb;
        this->numTriangles = t;
        for (float t : attr) {
            attributes.push_back(t);
        }
    }

};


#endif //BLENDSHAPESVIEWER_DRAWOBJECT_H
