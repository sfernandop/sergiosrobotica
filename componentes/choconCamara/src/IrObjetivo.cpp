// **********************************************************************
//
// Copyright (c) 2003-2013 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `IrObjetivo.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

#include <IrObjetivo.h>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <Ice/Object.h>
#include <IceUtil/Iterator.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 305
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

namespace
{

const ::std::string __RoboCompIrObjetivo__IrObjetivo__go_name = "go";

const ::std::string __RoboCompIrObjetivo__IrObjetivo__turn_name = "turn";

const ::std::string __RoboCompIrObjetivo__IrObjetivo__stop_name = "stop";

const ::std::string __RoboCompIrObjetivo__IrObjetivo__getState_name = "getState";

const ::std::string __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name = "cogerCaja";

const ::std::string __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name = "soltarCaja";

}
::IceProxy::Ice::Object* ::IceProxy::RoboCompIrObjetivo::upCast(::IceProxy::RoboCompIrObjetivo::IrObjetivo* p) { return p; }

void
::IceProxy::RoboCompIrObjetivo::__read(::IceInternal::BasicStream* __is, ::IceInternal::ProxyHandle< ::IceProxy::RoboCompIrObjetivo::IrObjetivo>& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::RoboCompIrObjetivo::IrObjetivo;
        v->__copyFrom(proxy);
    }
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::go(::Ice::Float x, ::Ice::Float z, const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__go_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            __del->go(x, z, __ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_go(::Ice::Float x, ::Ice::Float z, const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__go_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__go_name, ::Ice::Normal, __ctx);
        ::IceInternal::BasicStream* __os = __result->__startWriteParams(::Ice::DefaultFormat);
        __os->write(x);
        __os->write(z);
        __result->__endWriteParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_go(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompIrObjetivo__IrObjetivo__go_name);
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::turn(::Ice::Float speed, const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__turn_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            __del->turn(speed, __ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_turn(::Ice::Float speed, const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__turn_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__turn_name, ::Ice::Normal, __ctx);
        ::IceInternal::BasicStream* __os = __result->__startWriteParams(::Ice::DefaultFormat);
        __os->write(speed);
        __result->__endWriteParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_turn(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompIrObjetivo__IrObjetivo__turn_name);
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::stop(const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__stop_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            __del->stop(__ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_stop(const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__stop_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__stop_name, ::Ice::Normal, __ctx);
        __result->__writeEmptyParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_stop(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompIrObjetivo__IrObjetivo__stop_name);
}

::Ice::Float
IceProxy::RoboCompIrObjetivo::IrObjetivo::getState(const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__getState_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __checkTwowayOnly(__RoboCompIrObjetivo__IrObjetivo__getState_name);
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            return __del->getState(__ctx, __observer);
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_getState(const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    __checkAsyncTwowayOnly(__RoboCompIrObjetivo__IrObjetivo__getState_name);
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__getState_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__getState_name, ::Ice::Normal, __ctx);
        __result->__writeEmptyParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

::Ice::Float
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_getState(const ::Ice::AsyncResultPtr& __result)
{
    ::Ice::AsyncResult::__check(__result, this, __RoboCompIrObjetivo__IrObjetivo__getState_name);
    ::Ice::Float __ret;
    bool __ok = __result->__wait();
    try
    {
        if(!__ok)
        {
            try
            {
                __result->__throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                throw ::Ice::UnknownUserException(__FILE__, __LINE__, __ex.ice_name());
            }
        }
        ::IceInternal::BasicStream* __is = __result->__startReadParams();
        __is->read(__ret);
        __result->__endReadParams();
        return __ret;
    }
    catch(const ::Ice::LocalException& ex)
    {
        __result->__getObserver().failed(ex.ice_name());
        throw;
    }
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::cogerCaja(const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            __del->cogerCaja(__ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_cogerCaja(const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__cogerCaja_name, ::Ice::Normal, __ctx);
        __result->__writeEmptyParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_cogerCaja(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name);
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::soltarCaja(const ::Ice::Context* __ctx)
{
    ::IceInternal::InvocationObserver __observer(this, __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name, __ctx);
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate(false);
            ::IceDelegate::RoboCompIrObjetivo::IrObjetivo* __del = dynamic_cast< ::IceDelegate::RoboCompIrObjetivo::IrObjetivo*>(__delBase.get());
            __del->soltarCaja(__ctx, __observer);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex, __observer);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, true, __cnt, __observer);
        }
    }
}

::Ice::AsyncResultPtr
IceProxy::RoboCompIrObjetivo::IrObjetivo::begin_soltarCaja(const ::Ice::Context* __ctx, const ::IceInternal::CallbackBasePtr& __del, const ::Ice::LocalObjectPtr& __cookie)
{
    ::IceInternal::OutgoingAsyncPtr __result = new ::IceInternal::OutgoingAsync(this, __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name, __del, __cookie);
    try
    {
        __result->__prepare(__RoboCompIrObjetivo__IrObjetivo__soltarCaja_name, ::Ice::Normal, __ctx);
        __result->__writeEmptyParams();
        __result->__send(true);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __result->__exceptionAsync(__ex);
    }
    return __result;
}

void
IceProxy::RoboCompIrObjetivo::IrObjetivo::end_soltarCaja(const ::Ice::AsyncResultPtr& __result)
{
    __end(__result, __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name);
}

const ::std::string&
IceProxy::RoboCompIrObjetivo::IrObjetivo::ice_staticId()
{
    return ::RoboCompIrObjetivo::IrObjetivo::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::RoboCompIrObjetivo::IrObjetivo::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::RoboCompIrObjetivo::IrObjetivo);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::RoboCompIrObjetivo::IrObjetivo::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::RoboCompIrObjetivo::IrObjetivo);
}

::IceProxy::Ice::Object*
IceProxy::RoboCompIrObjetivo::IrObjetivo::__newInstance() const
{
    return new IrObjetivo;
}

void
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::go(::Ice::Float x, ::Ice::Float z, const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__go_name, ::Ice::Normal, __context, __observer);
    try
    {
        ::IceInternal::BasicStream* __os = __og.startWriteParams(::Ice::DefaultFormat);
        __os->write(x);
        __os->write(z);
        __og.endWriteParams();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::turn(::Ice::Float speed, const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__turn_name, ::Ice::Normal, __context, __observer);
    try
    {
        ::IceInternal::BasicStream* __os = __og.startWriteParams(::Ice::DefaultFormat);
        __os->write(speed);
        __og.endWriteParams();
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::stop(const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__stop_name, ::Ice::Normal, __context, __observer);
    __og.writeEmptyParams();
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

::Ice::Float
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::getState(const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__getState_name, ::Ice::Normal, __context, __observer);
    __og.writeEmptyParams();
    bool __ok = __og.invoke();
    ::Ice::Float __ret;
    try
    {
        if(!__ok)
        {
            try
            {
                __og.throwUserException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                throw __uue;
            }
        }
        ::IceInternal::BasicStream* __is = __og.startReadParams();
        __is->read(__ret);
        __og.endReadParams();
        return __ret;
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::cogerCaja(const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name, ::Ice::Normal, __context, __observer);
    __og.writeEmptyParams();
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateM::RoboCompIrObjetivo::IrObjetivo::soltarCaja(const ::Ice::Context* __context, ::IceInternal::InvocationObserver& __observer)
{
    ::IceInternal::Outgoing __og(__handler.get(), __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name, ::Ice::Normal, __context, __observer);
    __og.writeEmptyParams();
    bool __ok = __og.invoke();
    if(__og.hasResponse())
    {
        try
        {
            if(!__ok)
            {
                try
                {
                    __og.throwUserException();
                }
                catch(const ::Ice::UserException& __ex)
                {
                    ::Ice::UnknownUserException __uue(__FILE__, __LINE__, __ex.ice_name());
                    throw __uue;
                }
            }
            __og.readEmptyParams();
        }
        catch(const ::Ice::LocalException& __ex)
        {
            throw ::IceInternal::LocalExceptionWrapper(__ex, false);
        }
    }
}

void
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::go(::Ice::Float x, ::Ice::Float z, const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Float __p_x, ::Ice::Float __p_z, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_x(__p_x),
            _m_z(__p_z)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->go(_m_x, _m_z, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Float _m_x;
        ::Ice::Float _m_z;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__go_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(x, z, __current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::turn(::Ice::Float speed, const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Float __p_speed, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _m_speed(__p_speed)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->turn(_m_speed, _current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Float _m_speed;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__turn_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(speed, __current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::stop(const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->stop(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__stop_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::Ice::Float
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::getState(const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(::Ice::Float& __result, const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current),
            _result(__result)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            _result = servant->getState(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
        ::Ice::Float& _result;
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__getState_name, ::Ice::Normal, __context);
    ::Ice::Float __result;
    try
    {
        _DirectI __direct(__result, __current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
    return __result;
}

void
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::cogerCaja(const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->cogerCaja(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__cogerCaja_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

void
IceDelegateD::RoboCompIrObjetivo::IrObjetivo::soltarCaja(const ::Ice::Context* __context, ::IceInternal::InvocationObserver&)
{
    class _DirectI : public ::IceInternal::Direct
    {
    public:

        _DirectI(const ::Ice::Current& __current) : 
            ::IceInternal::Direct(__current)
        {
        }
        
        virtual ::Ice::DispatchStatus
        run(::Ice::Object* object)
        {
            ::RoboCompIrObjetivo::IrObjetivo* servant = dynamic_cast< ::RoboCompIrObjetivo::IrObjetivo*>(object);
            if(!servant)
            {
                throw ::Ice::OperationNotExistException(__FILE__, __LINE__, _current.id, _current.facet, _current.operation);
            }
            servant->soltarCaja(_current);
            return ::Ice::DispatchOK;
        }
        
    private:
        
    };
    
    ::Ice::Current __current;
    __initCurrent(__current, __RoboCompIrObjetivo__IrObjetivo__soltarCaja_name, ::Ice::Normal, __context);
    try
    {
        _DirectI __direct(__current);
        try
        {
            __direct.getServant()->__collocDispatch(__direct);
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
    }
    catch(const ::Ice::SystemException&)
    {
        throw;
    }
    catch(const ::IceInternal::LocalExceptionWrapper&)
    {
        throw;
    }
    catch(const ::std::exception& __ex)
    {
        ::IceInternal::LocalExceptionWrapper::throwWrapper(__ex);
    }
    catch(...)
    {
        throw ::IceInternal::LocalExceptionWrapper(::Ice::UnknownException(__FILE__, __LINE__, "unknown c++ exception"), false);
    }
}

::Ice::Object* RoboCompIrObjetivo::upCast(::RoboCompIrObjetivo::IrObjetivo* p) { return p; }

namespace
{
const ::std::string __RoboCompIrObjetivo__IrObjetivo_ids[2] =
{
    "::Ice::Object",
    "::RoboCompIrObjetivo::IrObjetivo"
};

}

bool
RoboCompIrObjetivo::IrObjetivo::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__RoboCompIrObjetivo__IrObjetivo_ids, __RoboCompIrObjetivo__IrObjetivo_ids + 2, _s);
}

::std::vector< ::std::string>
RoboCompIrObjetivo::IrObjetivo::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__RoboCompIrObjetivo__IrObjetivo_ids[0], &__RoboCompIrObjetivo__IrObjetivo_ids[2]);
}

const ::std::string&
RoboCompIrObjetivo::IrObjetivo::ice_id(const ::Ice::Current&) const
{
    return __RoboCompIrObjetivo__IrObjetivo_ids[1];
}

const ::std::string&
RoboCompIrObjetivo::IrObjetivo::ice_staticId()
{
    return __RoboCompIrObjetivo__IrObjetivo_ids[1];
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___go(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.startReadParams();
    ::Ice::Float x;
    ::Ice::Float z;
    __is->read(x);
    __is->read(z);
    __inS.endReadParams();
    go(x, z, __current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___turn(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.startReadParams();
    ::Ice::Float speed;
    __is->read(speed);
    __inS.endReadParams();
    turn(speed, __current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___stop(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.readEmptyParams();
    stop(__current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___getState(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.readEmptyParams();
    ::Ice::Float __ret = getState(__current);
    ::IceInternal::BasicStream* __os = __inS.__startWriteParams(::Ice::DefaultFormat);
    __os->write(__ret);
    __inS.__endWriteParams(true);
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___cogerCaja(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.readEmptyParams();
    cogerCaja(__current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::___soltarCaja(::IceInternal::Incoming& __inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    __inS.readEmptyParams();
    soltarCaja(__current);
    __inS.__writeEmptyParams();
    return ::Ice::DispatchOK;
}

namespace
{
const ::std::string __RoboCompIrObjetivo__IrObjetivo_all[] =
{
    "cogerCaja",
    "getState",
    "go",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping",
    "soltarCaja",
    "stop",
    "turn"
};

}

::Ice::DispatchStatus
RoboCompIrObjetivo::IrObjetivo::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< const ::std::string*, const ::std::string*> r = ::std::equal_range(__RoboCompIrObjetivo__IrObjetivo_all, __RoboCompIrObjetivo__IrObjetivo_all + 10, current.operation);
    if(r.first == r.second)
    {
        throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
    }

    switch(r.first - __RoboCompIrObjetivo__IrObjetivo_all)
    {
        case 0:
        {
            return ___cogerCaja(in, current);
        }
        case 1:
        {
            return ___getState(in, current);
        }
        case 2:
        {
            return ___go(in, current);
        }
        case 3:
        {
            return ___ice_id(in, current);
        }
        case 4:
        {
            return ___ice_ids(in, current);
        }
        case 5:
        {
            return ___ice_isA(in, current);
        }
        case 6:
        {
            return ___ice_ping(in, current);
        }
        case 7:
        {
            return ___soltarCaja(in, current);
        }
        case 8:
        {
            return ___stop(in, current);
        }
        case 9:
        {
            return ___turn(in, current);
        }
    }

    assert(false);
    throw ::Ice::OperationNotExistException(__FILE__, __LINE__, current.id, current.facet, current.operation);
}

void
RoboCompIrObjetivo::IrObjetivo::__writeImpl(::IceInternal::BasicStream* __os) const
{
    __os->startWriteSlice(ice_staticId(), -1, true);
    __os->endWriteSlice();
}

void
RoboCompIrObjetivo::IrObjetivo::__readImpl(::IceInternal::BasicStream* __is)
{
    __is->startReadSlice();
    __is->endReadSlice();
}

void 
RoboCompIrObjetivo::__patch(IrObjetivoPtr& handle, const ::Ice::ObjectPtr& v)
{
    handle = ::RoboCompIrObjetivo::IrObjetivoPtr::dynamicCast(v);
    if(v && !handle)
    {
        IceInternal::Ex::throwUOE(::RoboCompIrObjetivo::IrObjetivo::ice_staticId(), v);
    }
}
