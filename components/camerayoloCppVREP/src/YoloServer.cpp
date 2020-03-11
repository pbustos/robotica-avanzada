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

#include <YoloServer.h>
#include <IceUtil/PushDisableWarnings.h>
#include <Ice/LocalException.h>
#include <Ice/ValueFactory.h>
#include <Ice/OutgoingAsync.h>
#include <Ice/InputStream.h>
#include <Ice/OutputStream.h>
#include <Ice/LocalException.h>
#include <IceUtil/PopDisableWarnings.h>

#if defined(_MSC_VER)
#   pragma warning(disable:4458) // declaration of ... hides class member
#elif defined(__clang__)
#   pragma clang diagnostic ignored "-Wshadow"
#elif defined(__GNUC__)
#   pragma GCC diagnostic ignored "-Wshadow"
#endif

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

namespace
{

const ::IceInternal::DefaultUserExceptionFactoryInit<::RoboCompYoloServer::HardwareFailedException> iceC_RoboCompYoloServer_HardwareFailedException_init("::RoboCompYoloServer::HardwareFailedException");

const ::std::string iceC_RoboCompYoloServer_YoloServer_ids[2] =
{
    "::Ice::Object",
    "::RoboCompYoloServer::YoloServer"
};
const ::std::string iceC_RoboCompYoloServer_YoloServer_ops[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "processImage"
};
const ::std::string iceC_RoboCompYoloServer_YoloServer_processImage_name = "processImage";

}

RoboCompYoloServer::HardwareFailedException::~HardwareFailedException()
{
}

const ::std::string&
RoboCompYoloServer::HardwareFailedException::ice_staticId()
{
    static const ::std::string typeId = "::RoboCompYoloServer::HardwareFailedException";
    return typeId;
}

bool
RoboCompYoloServer::YoloServer::ice_isA(::std::string s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompYoloServer_YoloServer_ids, iceC_RoboCompYoloServer_YoloServer_ids + 2, s);
}

::std::vector<::std::string>
RoboCompYoloServer::YoloServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector<::std::string>(&iceC_RoboCompYoloServer_YoloServer_ids[0], &iceC_RoboCompYoloServer_YoloServer_ids[2]);
}

::std::string
RoboCompYoloServer::YoloServer::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompYoloServer::YoloServer::ice_staticId()
{
    static const ::std::string typeId = "::RoboCompYoloServer::YoloServer";
    return typeId;
}

/// \cond INTERNAL
bool
RoboCompYoloServer::YoloServer::_iceD_processImage(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::OperationMode::Normal, current.mode);
    auto istr = inS.startReadParams();
    TImage iceP_img;
    istr->readAll(iceP_img);
    inS.endReadParams();
    Objects ret = this->processImage(::std::move(iceP_img), current);
    auto ostr = inS.startWriteParams();
    ostr->writeAll(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

/// \cond INTERNAL
bool
RoboCompYoloServer::YoloServer::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompYoloServer_YoloServer_ops, iceC_RoboCompYoloServer_YoloServer_ops + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompYoloServer_YoloServer_ops)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_processImage(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}
/// \endcond

/// \cond INTERNAL
void
RoboCompYoloServer::YoloServerPrx::_iceI_processImage(const ::std::shared_ptr<::IceInternal::OutgoingAsyncT<::RoboCompYoloServer::Objects>>& outAsync, const TImage& iceP_img, const ::Ice::Context& context)
{
    _checkTwowayOnly(iceC_RoboCompYoloServer_YoloServer_processImage_name);
    outAsync->invoke(iceC_RoboCompYoloServer_YoloServer_processImage_name, ::Ice::OperationMode::Normal, ::Ice::FormatType::DefaultFormat, context,
        [&](::Ice::OutputStream* ostr)
        {
            ostr->writeAll(iceP_img);
        },
        [](const ::Ice::UserException& ex)
        {
            try
            {
                ex.ice_throw();
            }
            catch(const HardwareFailedException&)
            {
                throw;
            }
            catch(const ::Ice::UserException&)
            {
            }
        });
}
/// \endcond

/// \cond INTERNAL
::std::shared_ptr<::Ice::ObjectPrx>
RoboCompYoloServer::YoloServerPrx::_newInstance() const
{
    return ::IceInternal::createProxy<YoloServerPrx>();
}
/// \endcond

const ::std::string&
RoboCompYoloServer::YoloServerPrx::ice_staticId()
{
    return YoloServer::ice_staticId();
}

namespace Ice
{
}

#else // C++98 mapping

namespace
{

const ::std::string iceC_RoboCompYoloServer_YoloServer_processImage_name = "processImage";

}

namespace
{

const ::IceInternal::DefaultUserExceptionFactoryInit< ::RoboCompYoloServer::HardwareFailedException> iceC_RoboCompYoloServer_HardwareFailedException_init("::RoboCompYoloServer::HardwareFailedException");

}

RoboCompYoloServer::HardwareFailedException::HardwareFailedException(const ::std::string& what) :
    ::Ice::UserException(),
    what(what)
{
}

RoboCompYoloServer::HardwareFailedException::~HardwareFailedException() throw()
{
}

::std::string
RoboCompYoloServer::HardwareFailedException::ice_id() const
{
    return "::RoboCompYoloServer::HardwareFailedException";
}

RoboCompYoloServer::HardwareFailedException*
RoboCompYoloServer::HardwareFailedException::ice_clone() const
{
    return new HardwareFailedException(*this);
}

void
RoboCompYoloServer::HardwareFailedException::ice_throw() const
{
    throw *this;
}

/// \cond STREAM
void
RoboCompYoloServer::HardwareFailedException::_writeImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice("::RoboCompYoloServer::HardwareFailedException", -1, true);
    ::Ice::StreamWriter< HardwareFailedException, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
RoboCompYoloServer::HardwareFailedException::_readImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    ::Ice::StreamReader< HardwareFailedException, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}
/// \endcond

/// \cond INTERNAL
::IceProxy::Ice::Object* ::IceProxy::RoboCompYoloServer::upCast(YoloServer* p) { return p; }

void
::IceProxy::RoboCompYoloServer::_readProxy(::Ice::InputStream* istr, ::IceInternal::ProxyHandle< YoloServer>& v)
{
    ::Ice::ObjectPrx proxy;
    istr->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new YoloServer;
        v->_copyFrom(proxy);
    }
}
/// \endcond

::Ice::AsyncResultPtr
IceProxy::RoboCompYoloServer::YoloServer::_iceI_begin_processImage(const ::RoboCompYoloServer::TImage& iceP_img, const ::Ice::Context& context, const ::IceInternal::CallbackBasePtr& del, const ::Ice::LocalObjectPtr& cookie, bool sync)
{
    _checkTwowayOnly(iceC_RoboCompYoloServer_YoloServer_processImage_name, sync);
    ::IceInternal::OutgoingAsyncPtr result = new ::IceInternal::CallbackOutgoing(this, iceC_RoboCompYoloServer_YoloServer_processImage_name, del, cookie, sync);
    try
    {
        result->prepare(iceC_RoboCompYoloServer_YoloServer_processImage_name, ::Ice::Normal, context);
        ::Ice::OutputStream* ostr = result->startWriteParams(::Ice::DefaultFormat);
        ostr->write(iceP_img);
        result->endWriteParams();
        result->invoke(iceC_RoboCompYoloServer_YoloServer_processImage_name);
    }
    catch(const ::Ice::Exception& ex)
    {
        result->abort(ex);
    }
    return result;
}

::RoboCompYoloServer::Objects
IceProxy::RoboCompYoloServer::YoloServer::end_processImage(const ::Ice::AsyncResultPtr& result)
{
    ::Ice::AsyncResult::_check(result, this, iceC_RoboCompYoloServer_YoloServer_processImage_name);
    ::RoboCompYoloServer::Objects ret;
    if(!result->_waitForResponse())
    {
        try
        {
            result->_throwUserException();
        }
        catch(const ::RoboCompYoloServer::HardwareFailedException&)
        {
            throw;
        }
        catch(const ::Ice::UserException& ex)
        {
            throw ::Ice::UnknownUserException(__FILE__, __LINE__, ex.ice_id());
        }
    }
    ::Ice::InputStream* istr = result->_startReadParams();
    istr->read(ret);
    result->_endReadParams();
    return ret;
}

/// \cond INTERNAL
::IceProxy::Ice::Object*
IceProxy::RoboCompYoloServer::YoloServer::_newInstance() const
{
    return new YoloServer;
}
/// \endcond

const ::std::string&
IceProxy::RoboCompYoloServer::YoloServer::ice_staticId()
{
    return ::RoboCompYoloServer::YoloServer::ice_staticId();
}

RoboCompYoloServer::YoloServer::~YoloServer()
{
}

/// \cond INTERNAL
::Ice::Object* RoboCompYoloServer::upCast(YoloServer* p) { return p; }

/// \endcond

namespace
{
const ::std::string iceC_RoboCompYoloServer_YoloServer_ids[2] =
{
    "::Ice::Object",
    "::RoboCompYoloServer::YoloServer"
};

}

bool
RoboCompYoloServer::YoloServer::ice_isA(const ::std::string& s, const ::Ice::Current&) const
{
    return ::std::binary_search(iceC_RoboCompYoloServer_YoloServer_ids, iceC_RoboCompYoloServer_YoloServer_ids + 2, s);
}

::std::vector< ::std::string>
RoboCompYoloServer::YoloServer::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&iceC_RoboCompYoloServer_YoloServer_ids[0], &iceC_RoboCompYoloServer_YoloServer_ids[2]);
}

const ::std::string&
RoboCompYoloServer::YoloServer::ice_id(const ::Ice::Current&) const
{
    return ice_staticId();
}

const ::std::string&
RoboCompYoloServer::YoloServer::ice_staticId()
{
#ifdef ICE_HAS_THREAD_SAFE_LOCAL_STATIC
    static const ::std::string typeId = "::RoboCompYoloServer::YoloServer";
    return typeId;
#else
    return iceC_RoboCompYoloServer_YoloServer_ids[1];
#endif
}

/// \cond INTERNAL
bool
RoboCompYoloServer::YoloServer::_iceD_processImage(::IceInternal::Incoming& inS, const ::Ice::Current& current)
{
    _iceCheckMode(::Ice::Normal, current.mode);
    ::Ice::InputStream* istr = inS.startReadParams();
    TImage iceP_img;
    istr->read(iceP_img);
    inS.endReadParams();
    Objects ret = this->processImage(iceP_img, current);
    ::Ice::OutputStream* ostr = inS.startWriteParams();
    ostr->write(ret);
    inS.endWriteParams();
    return true;
}
/// \endcond

namespace
{
const ::std::string iceC_RoboCompYoloServer_YoloServer_all[] =
{
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "processImage"
};

}

/// \cond INTERNAL
bool
RoboCompYoloServer::YoloServer::_iceDispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair<const ::std::string*, const ::std::string*> r = ::std::equal_range(iceC_RoboCompYoloServer_YoloServer_all, iceC_RoboCompYoloServer_YoloServer_all + 5, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - iceC_RoboCompYoloServer_YoloServer_all)
    {
        case 0:
        {
            return _iceD_ice_id(in, current);
        }
        case 1:
        {
            return _iceD_ice_ids(in, current);
        }
        case 2:
        {
            return _iceD_ice_isA(in, current);
        }
        case 3:
        {
            return _iceD_ice_ping(in, current);
        }
        case 4:
        {
            return _iceD_processImage(in, current);
        }
        default:
        {
            assert(false);
            throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
        }
    }
}
/// \endcond

/// \cond STREAM
void
RoboCompYoloServer::YoloServer::_iceWriteImpl(::Ice::OutputStream* ostr) const
{
    ostr->startSlice(ice_staticId(), -1, true);
    ::Ice::StreamWriter< YoloServer, ::Ice::OutputStream>::write(ostr, *this);
    ostr->endSlice();
}

void
RoboCompYoloServer::YoloServer::_iceReadImpl(::Ice::InputStream* istr)
{
    istr->startSlice();
    ::Ice::StreamReader< YoloServer, ::Ice::InputStream>::read(istr, *this);
    istr->endSlice();
}
/// \endcond

/// \cond INTERNAL
void
RoboCompYoloServer::_icePatchObjectPtr(YoloServerPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = YoloServerPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(YoloServer::ice_staticId(), v);
    }
}
/// \endcond

namespace Ice
{
}

#endif
