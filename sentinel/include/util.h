#ifndef UTIL_H
#define UTIL_H
#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include <map>
#include <cmath>

template <class T>
class MPoint
{
public:
    T x, y;
    MPoint() : x(0), y(0) {}
    ~MPoint() { }
    template <class V>
    MPoint(const MPoint<V>& rhs): x(rhs.x),y(rhs.y) {}
    template <class V>
    MPoint(const V &_x, const V &_y) : x(_x), y(_y) {}
    template <class V>
    MPoint<T> &operator=(const MPoint<V> &rhs)
    {
        // if (this == &rhs)
        //     return *this;
        x = rhs.x;
        y = rhs.y;
        return *this;
    }
    template <class V>
    MPoint<T> operator+(const MPoint<V> &rhs) const
    {
        return MPoint<T>(x + rhs.x, y + rhs.y);
    }
    template <class V>
    MPoint<T> operator-(const MPoint<V> &rhs) const
    {
        return MPoint<T>(x - rhs.x, y - rhs.y);
    }
    template <class V>
    MPoint<T> operator*(V &&a) const
    {
        return MPoint<T>(x * a, y * a);
    }
    template <class V>
    MPoint<T> operator/(V &&a) const
    {
        return MPoint<T>(x / a, y / a);
    }
    double len() const
    {
        return sqrt(x * x + y * y);
    }
    template <class V>
    V lenSQ() const
    {
        return V(x * x + y * y);
    }
    template <class V>
    operator MPoint<V>() const
    {
        return MPoint<V>(x, y);
    }
};
typedef MPoint<int> IntPoint;
typedef MPoint<float> FloatPoint;
typedef MPoint<double> DoublePoint;

template <class T, class V>
bool operator==(const MPoint<T> &a, const MPoint<V> &b)
{
    return a.x == b.x && a.y == b.y;
}
template <class T, class V>
bool operator!=(const MPoint<T> &a, const MPoint<V> &b)
{
    return a.x != b.x || a.y != b.y;
}

#define SQ(x) ((x) * (x))

#endif