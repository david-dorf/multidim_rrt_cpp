// Shapes.h
#ifndef SHAPES_H
#define SHAPES_H

class Rectangle
{
public:
    Rectangle(float x, float y, float width, float height, float angle);

private:
    float x;
    float y;
    float width;
    float height;
    float angle;
};

class Sphere
{
public:
    Sphere(float x, float y, float z, float radius);

private:
    float x;
    float y;
    float z;
    float radius;
};

#endif // SHAPES_H