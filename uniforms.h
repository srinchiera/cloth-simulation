#ifndef UNIFORMS_H
#define UNIFORMS_H

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#if __GNUG__
#   include <tr1/memory>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "glsupport.h"
#include "texture.h"

// Private namespace for some helper functions. You should ignore this unless you
// are interested in the internal implementation.
namespace _helper {
inline void genericGlUniformi(GLint location, int i) {
  ::glUniform1i(location, i);
}
inline void genericGlUniformf(GLint location, float f) {
  ::glUniform1f(location, f);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<int, 1> *v) {
  ::glUniform1iv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<int, 2> *v) {
  ::glUniform2iv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<int, 3> *v) {
  ::glUniform3iv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<int, 4> *v) {
  ::glUniform4iv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<float, 1> *v) {
  ::glUniform1fv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<float, 2> *v) {
  ::glUniform2fv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<float, 3> *v) {
  ::glUniform3fv(location, size, &v[0][0]);
}
inline void genericGlUniformv(GLint location, int size, const Cvec<float, 4> *v) {
  ::glUniform4fv(location, size, &v[0][0]);
}
inline void genericGlUniformMatrix4(GLint location, const Matrix4& m) {
  GLfloat matf[16];
  m.writeToColumnMajorMatrix(matf);
  ::glUniformMatrix4fv(location, 1, GL_FALSE, matf);
}

template<typename T, int n>
inline GLenum getTypeForCvec();   // should replace with STATIC_ASSERT

template<>
inline GLenum getTypeForCvec<int, 1>() { return GL_INT; }
template<>
inline GLenum getTypeForCvec<int, 2>() { return GL_INT_VEC2; }
template<>
inline GLenum getTypeForCvec<int, 3>() { return GL_INT_VEC3; }
template<>
inline GLenum getTypeForCvec<int, 4>() { return GL_INT_VEC4; }
template<>
inline GLenum getTypeForCvec<float, 1>() { return GL_FLOAT; }
template<>
inline GLenum getTypeForCvec<float, 2>() { return GL_FLOAT_VEC2; }
template<>
inline GLenum getTypeForCvec<float, 3>() { return GL_FLOAT_VEC3; }
template<>
inline GLenum getTypeForCvec<float, 4>() { return GL_FLOAT_VEC4; }
template<>
inline GLenum getTypeForCvec<bool, 1>() { return GL_BOOL; }
template<>
inline GLenum getTypeForCvec<bool, 2>() { return GL_BOOL_VEC2; }
template<>
inline GLenum getTypeForCvec<bool, 3>() { return GL_BOOL_VEC3; }
template<>
inline GLenum getTypeForCvec<bool, 4>() { return GL_BOOL_VEC4; }
}

// The Uniforms keeps a map from strings to values
//
// Currently the value can be of the following type:
// - Single int, float, or Matrix4
// - Cvec<T, n> with T=int or float, and n = 1, 2, 3, or 4
// - shared_ptr<Texture>
//
// A Uniforms instance will start off empty, and you can use
// its put(name, value) member function to populate it.

class Uniforms {
public:
  Uniforms& put(const std::string& name, int value) {
    Cvec<int, 1> v(value);
    valueMap[name].reset(new CvecsValue<int, 1, 1>(&v));
    return *this;
  }

  Uniforms& put(const std::string& name, float value) {
    Cvec<float, 1> v(value);
    valueMap[name].reset(new CvecsValue<float, 1, 1>(&v));
    return *this;
  }

  Uniforms& put(const std::string& name, const Matrix4& value) {
    valueMap[name].reset(new Matrix4Value(value));
    return *this;
  }

  Uniforms& put(const std::string& name, const std::tr1::shared_ptr<Texture>& value) {
    valueMap[name].reset(new TextureValue(value));
    return *this;
  }

  template<int n>
  Uniforms& put(const std::string& name, const Cvec<int, n>& v) {
    valueMap[name].reset(new CvecsValue<int, n, 1>(&v));
    return *this;
  }

  template<int n>
  Uniforms& put(const std::string& name, const Cvec<float, n>& v) {
    valueMap[name].reset(new CvecsValue<float, n, 1>(&v));
    return *this;
  }

  template<int n>
  Uniforms& put(const std::string& name, const Cvec<double, n>& v) {
    Cvec<float, n> u;
    for (int i = 0; i < n; ++i) {
      u[i] = float(v[i]);
    }
    valueMap[name].reset(new CvecsValue<float, n, 1>(&u));
    return *this;
  }

  // Future work: add put for different sized matrices, and array of basic types
protected:

  // Ghastly implementation details follow. Viewer be warned.

  friend class Material;
  class ValueHolder;
  class Value;

  typedef std::map<std::string, ValueHolder> ValueMap;

  ValueMap valueMap;

  const Value* get(const std::string& name) const {
    std::map<std::string, ValueHolder>::const_iterator i = valueMap.find(name);
    return i == valueMap.end() ? NULL : i->second.get();
  }

  class ValueHolder {
    Value *value_;

public:
    ValueHolder() : value_(NULL) {}
    ValueHolder(Value* value) : value_(value) {}
    ValueHolder(const ValueHolder& u) : value_(u.value_ ? u.value_->clone() : NULL) {}
    ~ValueHolder() {
      if (value_)
        delete value_;
    }
    void reset(Value* value) {
      if (value_)
        delete value_;
      value_ = value;
    }

    Value *get() const {
      return value_;
    }

    ValueHolder& operator= (const ValueHolder& u) {
      reset(u.value_ ? u.value_->clone() : NULL);
      return *this;
    }
  };


  class Value {
public:
    GLenum type;
    GLint size;
    virtual Value* clone() const = 0;
    virtual const std::tr1::shared_ptr<Texture> * getTextures() const { return NULL; };
    virtual ~Value() {}

    // If type is one of GL_SAMPLER_*, the getTextures should provide a pointer to
    // the array of shared_ptr<Texture> stored by the uniform. And apply should
    // use the boundTexUnit argument as the argument for glUniform*.
    //
    // Otherwise, boundTexUnit should be ignored and the whatever value contained in
    // the Uniform instance should be set to given location.
    virtual void apply(GLint location, GLint boundTexUnit) const = 0;

protected:
    Value(GLenum aType, GLint aSize) : type(aType), size(aSize) {}
  };

  template<typename T, int n, int SIZE>
  class CvecsValue : public Value {
    Cvec<T, n> vs_[SIZE];

public:
    CvecsValue(const Cvec<T, n> vs[])
      : Value(_helper::getTypeForCvec<T,n>(), SIZE) {
      for (int i = 0; i < SIZE; ++i) {
        vs_[i] = vs[i];
      }
    }

    virtual Value* clone() const {
      return new CvecsValue(*this);
    }

    virtual void apply(GLint location, GLint boundTexUnit) const {
      _helper::genericGlUniformv(location, SIZE, &vs_[0]);
    }
  };

  class Matrix4Value : public Value {
    Matrix4 m_;
public:
    Matrix4Value(const Matrix4& m)
      : Value(GL_FLOAT_MAT4, 1)
      , m_(m) {}

    virtual Value* clone() const {
      return new Matrix4Value(*this);
    }

    virtual void apply(GLint location, GLint boundTexUnit) const {
      _helper::genericGlUniformMatrix4(location, m_);
    }
  };

  class TextureValue : public Value {
    std::tr1::shared_ptr<Texture> tex_;
public:
    TextureValue(const std::tr1::shared_ptr<Texture>& tex)
      : Value(tex->getSamplerType(), 1)
      , tex_(tex) {}

    virtual Value* clone() const {
      return new TextureValue(*this);
    }

    virtual void apply(GLint location, GLint boundTexUnit) const {
      _helper::genericGlUniformi(location, boundTexUnit);
    }

    virtual const std::tr1::shared_ptr<Texture> *getTextures() const {
      return &tex_;
    }
  };
};




#endif