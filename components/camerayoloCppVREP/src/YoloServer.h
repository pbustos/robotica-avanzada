//
// Copyright (c) ZeroC, Inc. All rights reserved.
//
//
// Ice version 3.7.3
//
// <auto-generated>
//
// Generated from file `YoloServer.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#ifndef __YoloServer_h__
#define __YoloServer_h__

#include <IceUtil/PushDisableWarnings.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/ValueF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/StreamHelpers.h>
#include <Ice/Comparable.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/GCObject.h>
#include <Ice/Value.h>
#include <Ice/Incoming.h>
#include <Ice/FactoryTableInit.h>
#include <IceUtil/ScopedArray.h>
#include <Ice/Optional.h>
#include <Ice/ExceptionHelpers.h>
#include <IceUtil/UndefSysMacros.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 307
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 >= 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 3
#       error Ice patch level mismatch!
#   endif
#endif

#ifdef ICE_CPP11_MAPPING // C++11 mapping

namespace RoboCompYoloServer
{

class YoloServer;
class YoloServerPrx;

}

namespace RoboCompYoloServer
{

class HardwareFailedException : public ::Ice::UserExceptionHelper<HardwareFailedException, ::Ice::UserException>
{
public:

    virtual ~HardwareFailedException();

    HardwareFailedException(const HardwareFailedException&) = default;

    HardwareFailedException() = default;

    /**
     * One-shot constructor to initialize all data members.
     */
    HardwareFailedException(const ::std::string& what) :
        what(::std::move(what))
    {
    }

    /**
     * Obtains a tuple containing all of the exception's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&> ice_tuple() const
    {
        return std::tie(what);
    }

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    ::std::string what;
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

using ImgType = ::std::vector<::Ice::Byte>;

struct TImage
{
    int width;
    int height;
    int depth;
    ::RoboCompYoloServer::ImgType image;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const int&, const int&, const int&, const ::RoboCompYoloServer::ImgType&> ice_tuple() const
    {
        return std::tie(width, height, depth, image);
    }
};

struct Box
{
    ::std::string name;
    int left;
    int top;
    int right;
    int bot;
    float prob;

    /**
     * Obtains a tuple containing all of the struct's data members.
     * @return The data members in a tuple.
     */
    std::tuple<const ::std::string&, const int&, const int&, const int&, const int&, const float&> ice_tuple() const
    {
        return std::tie(name, left, top, right, bot, prob);
    }
};

using Objects = ::std::vector<Box>;

using Ice::operator<;
using Ice::operator<=;
using Ice::operator>;
using Ice::operator>=;
using Ice::operator==;
using Ice::operator!=;

}

namespace RoboCompYoloServer
{

class YoloServer : public virtual ::Ice::Object
{
public:

    using ProxyType = YoloServerPrx;

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(::std::string id, const ::Ice::Current& current) const override;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector<::std::string> ice_ids(const ::Ice::Current& current) const override;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual ::std::string ice_id(const ::Ice::Current& current) const override;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual Objects processImage(TImage img, const ::Ice::Current& current) = 0;
    /// \cond INTERNAL
    bool _iceD_processImage(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&) override;
    /// \endcond
};

}

namespace RoboCompYoloServer
{

class YoloServerPrx : public virtual ::Ice::Proxy<YoloServerPrx, ::Ice::ObjectPrx>
{
public:

    Objects processImage(const TImage& img, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makePromiseOutgoing<::RoboCompYoloServer::Objects>(true, this, &YoloServerPrx::_iceI_processImage, img, context).get();
    }

    template<template<typename> class P = ::std::promise>
    auto processImageAsync(const TImage& img, const ::Ice::Context& context = ::Ice::noExplicitContext)
        -> decltype(::std::declval<P<::RoboCompYoloServer::Objects>>().get_future())
    {
        return _makePromiseOutgoing<::RoboCompYoloServer::Objects, P>(false, this, &YoloServerPrx::_iceI_processImage, img, context);
    }

    ::std::function<void()>
    processImageAsync(const TImage& img,
                      ::std::function<void(::RoboCompYoloServer::Objects)> response,
                      ::std::function<void(::std::exception_ptr)> ex = nullptr,
                      ::std::function<void(bool)> sent = nullptr,
                      const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _makeLamdaOutgoing<::RoboCompYoloServer::Objects>(response, ex, sent, this, &RoboCompYoloServer::YoloServerPrx::_iceI_processImage, img, context);
    }

    /// \cond INTERNAL
    void _iceI_processImage(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompYoloServer::Objects>>&, const TImage&, const ::Ice::Context&);
    /// \endcond

    /**
     * Obtains the Slice type ID of this interface.
     * @return The fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:

    /// \cond INTERNAL
    YoloServerPrx() = default;
    friend ::std::shared_ptr<YoloServerPrx> IceInternal::createProxy<YoloServerPrx>();

    virtual ::std::shared_ptr<::Ice::ObjectPrx> _newInstance() const override;
    /// \endcond
};

}

/// \cond STREAM
namespace Ice
{

template<typename S>
struct StreamReader<::RoboCompYoloServer::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompYoloServer::HardwareFailedException& v)
    {
        istr->readAll(v.what);
    }
};

template<>
struct StreamableTraits<::RoboCompYoloServer::TImage>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 13;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompYoloServer::TImage, S>
{
    static void read(S* istr, ::RoboCompYoloServer::TImage& v)
    {
        istr->readAll(v.width, v.height, v.depth, v.image);
    }
};

template<>
struct StreamableTraits<::RoboCompYoloServer::Box>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 21;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamReader<::RoboCompYoloServer::Box, S>
{
    static void read(S* istr, ::RoboCompYoloServer::Box& v)
    {
        istr->readAll(v.name, v.left, v.top, v.right, v.bot, v.prob);
    }
};

}
/// \endcond

/// \cond INTERNAL
namespace RoboCompYoloServer
{

using YoloServerPtr = ::std::shared_ptr<YoloServer>;
using YoloServerPrxPtr = ::std::shared_ptr<YoloServerPrx>;

}
/// \endcond

#else // C++98 mapping

namespace IceProxy
{

namespace RoboCompYoloServer
{

class YoloServer;
/// \cond INTERNAL
void _readProxy(::Ice::InputStream*, ::IceInternal::ProxyHandle< YoloServer>&);
::IceProxy::Ice::Object* upCast(YoloServer*);
/// \endcond

}

}

namespace RoboCompYoloServer
{

class YoloServer;
/// \cond INTERNAL
::Ice::Object* upCast(YoloServer*);
/// \endcond
typedef ::IceInternal::Handle< YoloServer> YoloServerPtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::RoboCompYoloServer::YoloServer> YoloServerPrx;
typedef YoloServerPrx YoloServerPrxPtr;
/// \cond INTERNAL
void _icePatchObjectPtr(YoloServerPtr&, const ::Ice::ObjectPtr&);
/// \endcond

}

namespace RoboCompYoloServer
{

class HardwareFailedException : public ::Ice::UserException
{
public:

    HardwareFailedException() {}
    /**
     * One-shot constructor to initialize all data members.
     */
    explicit HardwareFailedException(const ::std::string& what);
    virtual ~HardwareFailedException() throw();

    /**
     * Obtains the Slice type ID of this exception.
     * @return The fully-scoped type ID.
     */
    virtual ::std::string ice_id() const;
    /**
     * Polymporphically clones this exception.
     * @return A shallow copy of this exception.
     */
    virtual HardwareFailedException* ice_clone() const;
    /**
     * Throws this exception.
     */
    virtual void ice_throw() const;

    ::std::string what;

protected:

    /// \cond STREAM
    virtual void _writeImpl(::Ice::OutputStream*) const;
    virtual void _readImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
static HardwareFailedException _iceS_HardwareFailedException_init;
/// \endcond

typedef ::std::vector< ::Ice::Byte> ImgType;

struct TImage
{
    ::Ice::Int width;
    ::Ice::Int height;
    ::Ice::Int depth;
    ::RoboCompYoloServer::ImgType image;
};

struct Box
{
    ::std::string name;
    ::Ice::Int left;
    ::Ice::Int top;
    ::Ice::Int right;
    ::Ice::Int bot;
    ::Ice::Float prob;
};

typedef ::std::vector<Box> Objects;

}

namespace RoboCompYoloServer
{

/**
 * Base class for asynchronous callback wrapper classes used for calls to
 * IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 * Create a wrapper instance by calling ::RoboCompYoloServer::newCallback_YoloServer_processImage.
 */
class Callback_YoloServer_processImage_Base : public virtual ::IceInternal::CallbackBase { };
typedef ::IceUtil::Handle< Callback_YoloServer_processImage_Base> Callback_YoloServer_processImagePtr;

}

namespace IceProxy
{

namespace RoboCompYoloServer
{

class YoloServer : public virtual ::Ice::Proxy<YoloServer, ::IceProxy::Ice::Object>
{
public:

    ::RoboCompYoloServer::Objects processImage(const ::RoboCompYoloServer::TImage& img, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return end_processImage(_iceI_begin_processImage(img, context, ::IceInternal::dummyCallback, 0, true));
    }

    ::Ice::AsyncResultPtr begin_processImage(const ::RoboCompYoloServer::TImage& img, const ::Ice::Context& context = ::Ice::noExplicitContext)
    {
        return _iceI_begin_processImage(img, context, ::IceInternal::dummyCallback, 0);
    }

    ::Ice::AsyncResultPtr begin_processImage(const ::RoboCompYoloServer::TImage& img, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_processImage(img, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_processImage(const ::RoboCompYoloServer::TImage& img, const ::Ice::Context& context, const ::Ice::CallbackPtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_processImage(img, context, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_processImage(const ::RoboCompYoloServer::TImage& img, const ::RoboCompYoloServer::Callback_YoloServer_processImagePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_processImage(img, ::Ice::noExplicitContext, cb, cookie);
    }

    ::Ice::AsyncResultPtr begin_processImage(const ::RoboCompYoloServer::TImage& img, const ::Ice::Context& context, const ::RoboCompYoloServer::Callback_YoloServer_processImagePtr& cb, const ::Ice::LocalObjectPtr& cookie = 0)
    {
        return _iceI_begin_processImage(img, context, cb, cookie);
    }

    ::RoboCompYoloServer::Objects end_processImage(const ::Ice::AsyncResultPtr& result);

private:

    ::Ice::AsyncResultPtr _iceI_begin_processImage(const ::RoboCompYoloServer::TImage&, const ::Ice::Context&, const ::IceInternal::CallbackBasePtr&, const ::Ice::LocalObjectPtr& cookie = 0, bool sync = false);

public:

    /**
     * Obtains the Slice type ID corresponding to this interface.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

protected:
    /// \cond INTERNAL

    virtual ::IceProxy::Ice::Object* _newInstance() const;
    /// \endcond
};

}

}

namespace RoboCompYoloServer
{

class YoloServer : public virtual ::Ice::Object
{
public:

    typedef YoloServerPrx ProxyType;
    typedef YoloServerPtr PointerType;

    virtual ~YoloServer();

    /**
     * Determines whether this object supports an interface with the given Slice type ID.
     * @param id The fully-scoped Slice type ID.
     * @param current The Current object for the invocation.
     * @return True if this object supports the interface, false, otherwise.
     */
    virtual bool ice_isA(const ::std::string& id, const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a list of the Slice type IDs representing the interfaces supported by this object.
     * @param current The Current object for the invocation.
     * @return A list of fully-scoped type IDs.
     */
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains a Slice type ID representing the most-derived interface supported by this object.
     * @param current The Current object for the invocation.
     * @return A fully-scoped type ID.
     */
    virtual const ::std::string& ice_id(const ::Ice::Current& current = ::Ice::emptyCurrent) const;

    /**
     * Obtains the Slice type ID corresponding to this class.
     * @return A fully-scoped type ID.
     */
    static const ::std::string& ice_staticId();

    virtual Objects processImage(const TImage& img, const ::Ice::Current& current = ::Ice::emptyCurrent) = 0;
    /// \cond INTERNAL
    bool _iceD_processImage(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

    /// \cond INTERNAL
    virtual bool _iceDispatch(::IceInternal::Incoming&, const ::Ice::Current&);
    /// \endcond

protected:

    /// \cond STREAM
    virtual void _iceWriteImpl(::Ice::OutputStream*) const;
    virtual void _iceReadImpl(::Ice::InputStream*);
    /// \endcond
};

/// \cond INTERNAL
inline bool operator==(const YoloServer& lhs, const YoloServer& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) == static_cast<const ::Ice::Object&>(rhs);
}

inline bool operator<(const YoloServer& lhs, const YoloServer& rhs)
{
    return static_cast<const ::Ice::Object&>(lhs) < static_cast<const ::Ice::Object&>(rhs);
}
/// \endcond

}

/// \cond STREAM
namespace Ice
{

template<>
struct StreamableTraits< ::RoboCompYoloServer::HardwareFailedException>
{
    static const StreamHelperCategory helper = StreamHelperCategoryUserException;
};

template<typename S>
struct StreamWriter< ::RoboCompYoloServer::HardwareFailedException, S>
{
    static void write(S* ostr, const ::RoboCompYoloServer::HardwareFailedException& v)
    {
        ostr->write(v.what);
    }
};

template<typename S>
struct StreamReader< ::RoboCompYoloServer::HardwareFailedException, S>
{
    static void read(S* istr, ::RoboCompYoloServer::HardwareFailedException& v)
    {
        istr->read(v.what);
    }
};

template<>
struct StreamableTraits< ::RoboCompYoloServer::TImage>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 13;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompYoloServer::TImage, S>
{
    static void write(S* ostr, const ::RoboCompYoloServer::TImage& v)
    {
        ostr->write(v.width);
        ostr->write(v.height);
        ostr->write(v.depth);
        ostr->write(v.image);
    }
};

template<typename S>
struct StreamReader< ::RoboCompYoloServer::TImage, S>
{
    static void read(S* istr, ::RoboCompYoloServer::TImage& v)
    {
        istr->read(v.width);
        istr->read(v.height);
        istr->read(v.depth);
        istr->read(v.image);
    }
};

template<>
struct StreamableTraits< ::RoboCompYoloServer::Box>
{
    static const StreamHelperCategory helper = StreamHelperCategoryStruct;
    static const int minWireSize = 21;
    static const bool fixedLength = false;
};

template<typename S>
struct StreamWriter< ::RoboCompYoloServer::Box, S>
{
    static void write(S* ostr, const ::RoboCompYoloServer::Box& v)
    {
        ostr->write(v.name);
        ostr->write(v.left);
        ostr->write(v.top);
        ostr->write(v.right);
        ostr->write(v.bot);
        ostr->write(v.prob);
    }
};

template<typename S>
struct StreamReader< ::RoboCompYoloServer::Box, S>
{
    static void read(S* istr, ::RoboCompYoloServer::Box& v)
    {
        istr->read(v.name);
        istr->read(v.left);
        istr->read(v.top);
        istr->read(v.right);
        istr->read(v.bot);
        istr->read(v.prob);
    }
};

}
/// \endcond

namespace RoboCompYoloServer
{

/**
 * Type-safe asynchronous callback wrapper class used for calls to
 * IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 * Create a wrapper instance by calling ::RoboCompYoloServer::newCallback_YoloServer_processImage.
 */
template<class T>
class CallbackNC_YoloServer_processImage : public Callback_YoloServer_processImage_Base, public ::IceInternal::TwowayCallbackNC<T>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception&);
    typedef void (T::*Sent)(bool);
    typedef void (T::*Response)(const Objects&);

    CallbackNC_YoloServer_processImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallbackNC<T>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        YoloServerPrx proxy = YoloServerPrx::uncheckedCast(result->getProxy());
        Objects ret;
        try
        {
            ret = proxy->end_processImage(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::CallbackNC<T>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::CallbackNC<T>::_callback.get()->*_response)(ret);
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 */
template<class T> Callback_YoloServer_processImagePtr
newCallback_YoloServer_processImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const Objects&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_YoloServer_processImage<T>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 */
template<class T> Callback_YoloServer_processImagePtr
newCallback_YoloServer_processImage(T* instance, void (T::*cb)(const Objects&), void (T::*excb)(const ::Ice::Exception&), void (T::*sentcb)(bool) = 0)
{
    return new CallbackNC_YoloServer_processImage<T>(instance, cb, excb, sentcb);
}

/**
 * Type-safe asynchronous callback wrapper class with cookie support used for calls to
 * IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 * Create a wrapper instance by calling ::RoboCompYoloServer::newCallback_YoloServer_processImage.
 */
template<class T, typename CT>
class Callback_YoloServer_processImage : public Callback_YoloServer_processImage_Base, public ::IceInternal::TwowayCallback<T, CT>
{
public:

    typedef IceUtil::Handle<T> TPtr;

    typedef void (T::*Exception)(const ::Ice::Exception& , const CT&);
    typedef void (T::*Sent)(bool , const CT&);
    typedef void (T::*Response)(const Objects&, const CT&);

    Callback_YoloServer_processImage(const TPtr& obj, Response cb, Exception excb, Sent sentcb)
        : ::IceInternal::TwowayCallback<T, CT>(obj, cb != 0, excb, sentcb), _response(cb)
    {
    }

    /// \cond INTERNAL
    virtual void completed(const ::Ice::AsyncResultPtr& result) const
    {
        YoloServerPrx proxy = YoloServerPrx::uncheckedCast(result->getProxy());
        Objects ret;
        try
        {
            ret = proxy->end_processImage(result);
        }
        catch(const ::Ice::Exception& ex)
        {
            ::IceInternal::Callback<T, CT>::exception(result, ex);
            return;
        }
        if(_response)
        {
            (::IceInternal::Callback<T, CT>::_callback.get()->*_response)(ret, CT::dynamicCast(result->getCookie()));
        }
    }
    /// \endcond

private:

    Response _response;
};

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 */
template<class T, typename CT> Callback_YoloServer_processImagePtr
newCallback_YoloServer_processImage(const IceUtil::Handle<T>& instance, void (T::*cb)(const Objects&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_YoloServer_processImage<T, CT>(instance, cb, excb, sentcb);
}

/**
 * Creates a callback wrapper instance that delegates to your object.
 * Use this overload when your callback methods receive a cookie value.
 * @param instance The callback object.
 * @param cb The success method of the callback object.
 * @param excb The exception method of the callback object.
 * @param sentcb The sent method of the callback object.
 * @return An object that can be passed to an asynchronous invocation of IceProxy::RoboCompYoloServer::YoloServer::begin_processImage.
 */
template<class T, typename CT> Callback_YoloServer_processImagePtr
newCallback_YoloServer_processImage(T* instance, void (T::*cb)(const Objects&, const CT&), void (T::*excb)(const ::Ice::Exception&, const CT&), void (T::*sentcb)(bool, const CT&) = 0)
{
    return new Callback_YoloServer_processImage<T, CT>(instance, cb, excb, sentcb);
}

}

#endif

#include <IceUtil/PopDisableWarnings.h>
#endif
