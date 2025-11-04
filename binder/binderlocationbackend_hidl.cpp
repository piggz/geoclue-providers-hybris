/*
    Copyright (c) 2015 - 2021 Jolla Ltd.
    Copyright (c) 2018 Matti Lehtimäki <matti.lehtimaki@gmail.com>
    Copyright (c) 2025 Jolla Mobile Ltd

    This file is part of geoclue-hybris.

    Geoclue-hybris is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License.
*/

#include "binderlocationbackend_hidl.h"

#include "hybrisprovider.h"

#include <QtNetwork/QHostAddress>

#define GNSS_BINDER_DEFAULT_DEV  "/dev/hwbinder"

enum GnssFunctions {
    GNSS_SET_CALLBACK = 1,
    GNSS_START = 2,
    GNSS_STOP = 3,
    GNSS_CLEANUP = 4,
    GNSS_INJECT_TIME = 5,
    GNSS_INJECT_LOCATION = 6,
    GNSS_DELETE_AIDING_DATA = 7,
    GNSS_SET_POSITION_MODE = 8,
    GNSS_GET_EXTENSION_AGNSS_RIL = 9,
    GNSS_GET_EXTENSION_GNSS_GEOFENCING = 10,
    GNSS_GET_EXTENSION_AGNSS = 11,
    GNSS_GET_EXTENSION_GNSS_NI = 12,
    GNSS_GET_EXTENSION_GNSS_MEASUREMENT = 13,
    GNSS_GET_EXTENSION_GNSS_NAVIGATION_MESSAGE = 14,
    GNSS_GET_EXTENSION_XTRA = 15,
    GNSS_GET_EXTENSION_GNSS_CONFIGURATION = 16,
    GNSS_GET_EXTENSION_GNSS_DEBUG = 17,
    GNSS_GET_EXTENSION_GNSS_BATCHING = 18
};

enum GnssCallbacks {
    GNSS_LOCATION_CB = 1,
    GNSS_STATUS_CB = 2,
    GNSS_SV_STATUS_CB = 3,
    GNSS_NMEA_CB = 4,
    GNSS_SET_CAPABILITIES_CB = 5,
    GNSS_ACQUIRE_WAKELOCK_CB = 6,
    GNSS_RELEASE_WAKELOCK_CB = 7,
    GNSS_REQUEST_TIME_CB = 8,
    GNSS_SET_SYSTEM_INFO_CB = 9
};

enum GnssDebugFunctions {
    GNSS_DEBUG_GET_DEBUG_DATA = 1
};

enum GnssNiFunctions {
    GNSS_NI_SET_CALLBACK = 1,
    GNSS_NI_RESPOND = 2
};

enum GnssNiCallbacks {
    GNSS_NI_NOTIFY_CB = 1
};

enum GnssXtraFunctions {
    GNSS_XTRA_SET_CALLBACK = 1,
    GNSS_XTRA_INJECT_XTRA_DATA = 2
};

enum GnssXtraCallbacks {
    GNSS_XTRA_DOWNLOAD_REQUEST_CB = 1
};

enum AGnssFunctions {
    AGNSS_SET_CALLBACK = 1,
    AGNSS_DATA_CONN_CLOSED = 2,
    AGNSS_DATA_CONN_FAILED = 3,
    AGNSS_SET_SERVER = 4,
    AGNSS_DATA_CONN_OPEN = 5
};

enum AGnssCallbacks {
    AGNSS_STATUS_IP_V4_CB = 1,
    AGNSS_STATUS_IP_V6_CB = 2
};

enum AGnssRilFunctions {
    AGNSS_RIL_SET_CALLBACK = 1,
    AGNSS_RIL_SET_REF_LOCATION = 2,
    AGNSS_RIL_SET_ID = 3,
    AGNSS_RIL_UPDATE_NETWORK_STATE = 4,
    AGNSS_RIL_UPDATE_NETWORK_AVAILABILITY = 5
};

enum AGnssRilCallbacks {
    AGNSS_RIL_REQUEST_REF_ID_CB = 1,
    AGNSS_RIL_REQUEST_REF_LOC_CB = 2
};

#define GNSS_IFACE(x)       "android.hardware.gnss@1.0::" x
#define GNSS_REMOTE         GNSS_IFACE("IGnss")
#define GNSS_CALLBACK       GNSS_IFACE("IGnssCallback")
#define GNSS_DEBUG_REMOTE   GNSS_IFACE("IGnssDebug")
#define GNSS_NI_REMOTE      GNSS_IFACE("IGnssNi")
#define GNSS_NI_CALLBACK    GNSS_IFACE("IGnssNiCallback")
#define GNSS_XTRA_REMOTE    GNSS_IFACE("IGnssXtra")
#define GNSS_XTRA_CALLBACK  GNSS_IFACE("IGnssXtraCallback")
#define AGNSS_REMOTE        GNSS_IFACE("IAGnss")
#define AGNSS_CALLBACK      GNSS_IFACE("IAGnssCallback")
#define AGNSS_RIL_REMOTE    GNSS_IFACE("IAGnssRil")
#define AGNSS_RIL_CALLBACK  GNSS_IFACE("IAGnssRilCallback")


/*==========================================================================*
 * Implementation
 *==========================================================================*/

namespace
{

const double MpsToKnots = 1.943844;

GBinderLocalReply *geoclue_binder_gnss_callback(
    GBinderLocalObject *obj,
    GBinderRemoteRequest *req,
    guint code,
    guint flags,
    int *status,
    void *user_data)
{
    Q_UNUSED(flags)
    Q_UNUSED(user_data)
    const char *iface = gbinder_remote_request_interface(req);

    if (!g_strcmp0(iface, GNSS_CALLBACK)) {
        GBinderReader reader;
        gbinder_remote_request_init_reader(req, &reader);
        switch (code) {
        case GNSS_LOCATION_CB:
            {
            Location loc;

            const GnssLocation *location = geoclue_binder_gnss_decode_struct
                (GnssLocation, &reader);

            loc.setTimestamp(location->timestamp);

            if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_LAT_LONG) {
                loc.setLatitude(location->latitudeDegrees);
                loc.setLongitude(location->longitudeDegrees);
            }

            if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_ALTITUDE)
                loc.setAltitude(location->altitudeMeters);

            if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_SPEED)
                loc.setSpeed(location->speedMetersPerSec * MpsToKnots);

            if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_BEARING)
                loc.setDirection(location->bearingDegrees);

            if ((location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_HORIZONTAL_ACCURACY) ||
                (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_VERTICAL_ACCURACY)) {
                Accuracy accuracy;
                if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_HORIZONTAL_ACCURACY) {
                    accuracy.setHorizontal(location->horizontalAccuracyMeters);
                }
                if (location->gnssLocationFlags & HYBRIS_GNSS_LOCATION_HAS_VERTICAL_ACCURACY) {
                    accuracy.setVertical(location->verticalAccuracyMeters);
                }
                loc.setAccuracy(accuracy);
            }

            QMetaObject::invokeMethod(staticProvider, "setLocation", Qt::QueuedConnection,
                                      Q_ARG(Location, loc));
            }
            break;
        case GNSS_STATUS_CB:
            {
            guint32 stat;
            if (gbinder_reader_read_uint32(&reader, &stat)) {
                if (stat == HYBRIS_GNSS_STATUS_ENGINE_ON) {
                    QMetaObject::invokeMethod(staticProvider, "engineOn", Qt::QueuedConnection);
                }
                if (stat == HYBRIS_GNSS_STATUS_ENGINE_OFF) {
                    QMetaObject::invokeMethod(staticProvider, "engineOff", Qt::QueuedConnection);
                }
            }
            }
            break;
        case GNSS_SV_STATUS_CB:
            {
            const GnssSvStatus *svStatus = geoclue_binder_gnss_decode_struct
                (GnssSvStatus, &reader);

            QList<SatelliteInfo> satellites;
            QList<int> usedPrns;

            for (int i = 0; i < svStatus->numSvs; ++i) {
                SatelliteInfo satInfo;
                GnssSvInfo svInfo = svStatus->gnssSvList[i];
                satInfo.setSnr(svInfo.cN0Dbhz);
                satInfo.setElevation(svInfo.elevationDegrees);
                satInfo.setAzimuth(svInfo.azimuthDegrees);
                int prn = svInfo.svid;
                // From https://github.com/barbeau/gpstest
                // and https://github.com/mvglasow/satstat/wiki/NMEA-IDs
                if (svInfo.constellation == GnssConstellationType::SBAS) {
                    prn -= 87;
                } else if (svInfo.constellation == GnssConstellationType::GLONASS) {
                    prn += 64;
                } else if (svInfo.constellation == GnssConstellationType::BEIDOU) {
                    prn += 200;
                } else if (svInfo.constellation == GnssConstellationType::GALILEO) {
                    prn += 300;
                }
                satInfo.setPrn(prn);
                satellites.append(satInfo);

                if (svInfo.svFlag & HYBRIS_GNSS_SV_FLAGS_USED_IN_FIX)
                    usedPrns.append(prn);
            }

            QMetaObject::invokeMethod(staticProvider, "setSatellite", Qt::QueuedConnection,
                                      Q_ARG(QList<SatelliteInfo>, satellites),
                                      Q_ARG(QList<int>, usedPrns));
            }
            break;
        case GNSS_NMEA_CB:
            {
            gint64 timestamp;
            if (gbinder_reader_read_int64(&reader, &timestamp)) {
                char *nmeaData = gbinder_reader_read_hidl_string(&reader);
                if (nmeaData) {
                    processNmea(timestamp, nmeaData);
                    g_free(nmeaData);
                }
            }
            }
            break;
        case GNSS_SET_CAPABILITIES_CB:
            {
            guint32 capabilities;
            if (gbinder_reader_read_uint32(&reader, &capabilities)) {
                qCDebug(lcGeoclueHybris) << "capabilities" << showbase << hex << capabilities;
            }
            }
            break;
        case GNSS_ACQUIRE_WAKELOCK_CB:
        case GNSS_RELEASE_WAKELOCK_CB:
            break;
        case GNSS_REQUEST_TIME_CB:
            qCDebug(lcGeoclueHybris) << "GNSS request UTC time";
            QMetaObject::invokeMethod(staticProvider, "injectUtcTime", Qt::QueuedConnection);
            break;
        case GNSS_SET_SYSTEM_INFO_CB:
            qCDebug(lcGeoclueHybris) << "GNSS set system info";
            break;
        default:
            qWarning("Failed to decode callback %u", code);
            break;
        }
        *status = GBINDER_STATUS_OK;
        return gbinder_local_reply_append_int32(gbinder_local_object_new_reply(obj), 0);
    } else {
        qWarning("Unknown interface %s and code %u", iface, code);
        *status = GBINDER_STATUS_FAILED;
    }
    return Q_NULLPTR;
}

GBinderLocalReply *geoclue_binder_gnss_xtra_callback(
    GBinderLocalObject *obj,
    GBinderRemoteRequest *req,
    guint code,
    guint flags,
    int *status,
    void *user_data)
{
    Q_UNUSED(flags)
    Q_UNUSED(user_data)
    const char *iface = gbinder_remote_request_interface(req);

    if (!g_strcmp0(iface, GNSS_XTRA_CALLBACK)) {
        GBinderReader reader;

        gbinder_remote_request_init_reader(req, &reader);
        switch (code) {
        case GNSS_XTRA_DOWNLOAD_REQUEST_CB:
            QMetaObject::invokeMethod(staticProvider, "xtraDownloadRequest", Qt::QueuedConnection);
            break;
        default:
            qWarning("Failed to decode callback %u", code);
            break;
        }
        *status = GBINDER_STATUS_OK;
        return gbinder_local_reply_append_int32(gbinder_local_object_new_reply(obj), 0);
    } else {
        qWarning("Unknown interface %s and code %u", iface, code);
        *status = GBINDER_STATUS_FAILED;
    }
    return Q_NULLPTR;
}

GBinderLocalReply *geoclue_binder_agnss_callback(
    GBinderLocalObject *obj,
    GBinderRemoteRequest *req,
    guint code,
    guint flags,
    int *status,
    void *user_data)
{
    Q_UNUSED(flags)
    Q_UNUSED(user_data)
    const char *iface = gbinder_remote_request_interface(req);

    if (!g_strcmp0(iface, AGNSS_CALLBACK)) {
        GBinderReader reader;

        gbinder_remote_request_init_reader(req, &reader);
        switch (code) {
        case AGNSS_STATUS_IP_V4_CB:
            {
            QHostAddress ipv4;
            QHostAddress ipv6;
            QByteArray ssid;
            QByteArray password;

            const AGnssStatusIpV4 *status = geoclue_binder_gnss_decode_struct
                (AGnssStatusIpV4, &reader);

            ipv4.setAddress(status->ipV4Addr);

            QMetaObject::invokeMethod(staticProvider, "agpsStatus", Qt::QueuedConnection,
                                      Q_ARG(qint16, status->type), Q_ARG(quint16, status->status),
                                      Q_ARG(QHostAddress, ipv4), Q_ARG(QHostAddress, ipv6),
                                      Q_ARG(QByteArray, ssid), Q_ARG(QByteArray, password));
            }
            break;
        case AGNSS_STATUS_IP_V6_CB:
            {
            QHostAddress ipv4;
            QHostAddress ipv6;
            QByteArray ssid;
            QByteArray password;

            const AGnssStatusIpV6 *status = geoclue_binder_gnss_decode_struct
                (AGnssStatusIpV6, &reader);

            ipv6.setAddress(status->ipV6Addr);

            QMetaObject::invokeMethod(staticProvider, "agpsStatus", Qt::QueuedConnection,
                                      Q_ARG(qint16, status->type), Q_ARG(quint16, status->status),
                                      Q_ARG(QHostAddress, ipv4), Q_ARG(QHostAddress, ipv6),
                                      Q_ARG(QByteArray, ssid), Q_ARG(QByteArray, password));
            }
            break;
        default:
            qWarning("Failed to decode callback %u", code);
            break;
        }
        *status = GBINDER_STATUS_OK;
        return gbinder_local_reply_append_int32(gbinder_local_object_new_reply(obj), 0);
    } else {
        qWarning("Unknown interface %s and code %u", iface, code);
        *status = GBINDER_STATUS_FAILED;
    }
    return Q_NULLPTR;
}


GBinderLocalReply *geoclue_binder_agnss_ril_callback(
    GBinderLocalObject *obj,
    GBinderRemoteRequest *req,
    guint code,
    guint flags,
    int *status,
    void *user_data)
{
    Q_UNUSED(flags)
    Q_UNUSED(user_data)
    const char *iface = gbinder_remote_request_interface(req);

    if (!g_strcmp0(iface, AGNSS_RIL_CALLBACK)) {
        GBinderReader reader;

        gbinder_remote_request_init_reader(req, &reader);
        switch (code) {
        case AGNSS_RIL_REQUEST_REF_ID_CB:
            qCDebug(lcGeoclueHybris) << "AGNSS RIL request ref ID";
            break;
        case AGNSS_RIL_REQUEST_REF_LOC_CB:
            qCDebug(lcGeoclueHybris) << "AGNSS RIL request ref location";
            break;
        default:
            qWarning("Failed to decode callback %u", code);
            break;
        }
        *status = GBINDER_STATUS_OK;
        return gbinder_local_reply_append_int32(gbinder_local_object_new_reply(obj), 0);
    } else {
        qWarning("Unknown interface %s and code %u", iface, code);
        *status = GBINDER_STATUS_FAILED;
    }
    return Q_NULLPTR;
}


GBinderLocalReply *geoclue_binder_gnss_ni_callback(
    GBinderLocalObject *obj,
    GBinderRemoteRequest *req,
    guint code,
    guint flags,
    int *status,
    void *user_data)
{
    Q_UNUSED(flags)
    Q_UNUSED(user_data)
    const char *iface = gbinder_remote_request_interface(req);

    if (!g_strcmp0(iface, GNSS_NI_CALLBACK)) {
        GBinderReader reader;

        gbinder_remote_request_init_reader(req, &reader);
        switch (code) {
        case GNSS_NI_NOTIFY_CB:
            qCDebug(lcGeoclueHybris) << "GNSS NI notify";
            break;
        default:
            qWarning("Failed to decode callback %u", code);
            break;
        }
        *status = GBINDER_STATUS_OK;
        return gbinder_local_reply_append_int32(gbinder_local_object_new_reply(obj), 0);
    } else {
        qWarning("Unknown interface %s and code %u", iface, code);
        *status = GBINDER_STATUS_FAILED;
    }
    return Q_NULLPTR;
}

void geoclue_binder_gnss_gnss_died(
    GBinderRemoteObject */*obj*/,
    void *user_data)
{
    BinderLocationBackendHidl *self = (BinderLocationBackendHidl *)user_data;
    self->dropGnss();
}

}

/*==========================================================================*
 * Backend class
 *==========================================================================*/

BinderLocationBackendHidl::BinderLocationBackendHidl(QObject *parent)
:   BinderLocationBackend(parent)
{
}

bool BinderLocationBackendHidl::isSupported()
{

    bool ret = false;
    GBinderServiceManager *sm =
        gbinder_servicemanager_new(GNSS_BINDER_DEFAULT_DEV);

    if (sm) {
        /* Fetch remote reference from hwservicemanager */
        char *fqname = g_strconcat(GNSS_REMOTE "/default", Q_NULLPTR);
        GBinderRemoteObject *remoteGnss =
            gbinder_servicemanager_get_service_sync(sm, fqname, NULL);

        if (remoteGnss) {
            ret = true;
        }
        g_free(fqname);
    }

    gbinder_servicemanager_unref(sm);

    return ret;
}

bool BinderLocationBackendHidl::isReplySuccess(GBinderRemoteReply *reply)
{
    GBinderReader reader;
    gint32 status;
    gboolean result;

    if (!reply) {
        return false;
    }

    gbinder_remote_reply_init_reader(reply, &reader);

    if (!gbinder_reader_read_int32(&reader, &status) || status != 0) {
        return false;
    }
    if (!gbinder_reader_read_bool(&reader, &result) || !result) {
        return false;
    }

    return true;
}

// Gnss
bool BinderLocationBackendHidl::gnssInit()
{
    bool ret = false;

    qWarning("Initialising GNSS interface");

    m_sm = gbinder_servicemanager_new(GNSS_BINDER_DEFAULT_DEV);
    if (m_sm) {
        int status = 0;

        /* Fetch remote reference from hwservicemanager */
        m_fqname = g_strconcat(GNSS_REMOTE "/default", Q_NULLPTR);
        m_remoteGnss = gbinder_servicemanager_get_service_sync(m_sm,
            m_fqname, &status);

        if (m_remoteGnss) {
            GBinderLocalRequest *req;
            GBinderRemoteReply *reply;

            /* get_service returns auto-released reference,
             * we need to add a reference of our own */
            gbinder_remote_object_ref(m_remoteGnss);
            m_clientGnss = gbinder_client_new(m_remoteGnss, GNSS_REMOTE);
            m_death_id = gbinder_remote_object_add_death_handler
                (m_remoteGnss, geoclue_binder_gnss_gnss_died, this);
            m_callbackGnss = gbinder_servicemanager_new_local_object
                (m_sm, GNSS_CALLBACK, geoclue_binder_gnss_callback, this);

            /* IGnss::setCallback */
            req = gbinder_client_new_request(m_clientGnss);
            gbinder_local_request_append_local_object(req, m_callbackGnss);
            reply = gbinder_client_transact_sync_reply(m_clientGnss,
                GNSS_SET_CALLBACK, req, &status);

            if (!status && isReplySuccess(reply)) {
                ret = true;
            }

            gbinder_local_request_unref(req);
            gbinder_remote_reply_unref(reply);
        }
    }

    if (!ret) {
        qWarning("Failed to initialise GNSS interface");
    }
    return ret;
}

bool BinderLocationBackendHidl::gnssStart()
{
    bool ret = false;

    if (m_clientGnss) {
        int status = 0;
        GBinderRemoteReply *reply;

        reply = gbinder_client_transact_sync_reply(m_clientGnss,
            GNSS_START, Q_NULLPTR, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }

        gbinder_remote_reply_unref(reply);
    }

    if (!ret) {
        qWarning("Failed to start positioning");
    }
    return ret;
}

bool BinderLocationBackendHidl::gnssStop()
{
    bool ret = false;

    if (m_clientGnss) {
        int status = 0;
        GBinderRemoteReply *reply;

        reply = gbinder_client_transact_sync_reply(m_clientGnss,
            GNSS_STOP, Q_NULLPTR, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }

        gbinder_remote_reply_unref(reply);
    }

    if (!ret) {
        qWarning("Failed to stop positioning");
    }
    return ret;
}

void BinderLocationBackendHidl::gnssCleanup()
{
    if (m_clientGnss) {
        gbinder_client_transact(m_clientGnss, GNSS_CLEANUP, 0,
            NULL, NULL, NULL, NULL);
    }
}

bool BinderLocationBackendHidl::gnssInjectLocation(
    int timestamp,
    double latitudeDegrees,
    double longitudeDegrees,
    float accuracyMeters)
{
    Q_UNUSED(timestamp)
    bool ret = false;

    if (m_clientGnss) {
        int status = 0;

        GBinderLocalRequest *req;
        GBinderRemoteReply *reply;
        GBinderWriter writer;

        req = gbinder_client_new_request(m_clientGnss);
        gbinder_local_request_init_writer(req, &writer);
        gbinder_writer_append_double(&writer, latitudeDegrees);
        gbinder_writer_append_double(&writer, longitudeDegrees);
        gbinder_writer_append_float(&writer, accuracyMeters);
        reply = gbinder_client_transact_sync_reply(m_clientGnss,
            GNSS_INJECT_LOCATION, req, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }
        if (!ret) {
            qWarning("Failed to inject location");
        }

        gbinder_local_request_unref(req);
        gbinder_remote_reply_unref(reply);
    }
    return ret;
}

bool BinderLocationBackendHidl::gnssInjectTime(
    HybrisGnssUtcTime timeMs,
    int64_t timeReferenceMs,
    int32_t uncertaintyMs)
{
    bool ret = false;

    if (m_clientGnss) {
        int status = 0;
        GBinderLocalRequest *req;
        GBinderRemoteReply *reply;
        GBinderWriter writer;

        req = gbinder_client_new_request(m_clientGnss);
        gbinder_local_request_init_writer(req, &writer);
        gbinder_writer_append_int64(&writer, timeMs);
        gbinder_writer_append_int64(&writer, timeReferenceMs);
        gbinder_writer_append_int32(&writer, uncertaintyMs);

        reply = gbinder_client_transact_sync_reply(m_clientGnss,
            GNSS_INJECT_TIME, req, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }

        if (!ret) {
            qWarning("Failed to inject time");
        }
        gbinder_local_request_unref(req);
        gbinder_remote_reply_unref(reply);
    }
    return ret;
}

void BinderLocationBackendHidl::gnssDeleteAidingData(
    HybrisGnssAidingData aidingDataFlags)
{
    if (m_clientGnss) {
        GBinderLocalRequest *req;

        req = gbinder_client_new_request(m_clientGnss);
        gbinder_local_request_append_int32(req, aidingDataFlags);
        gbinder_client_transact(m_clientGnss, GNSS_DELETE_AIDING_DATA,
                                0, req, NULL, NULL, NULL);

        gbinder_local_request_unref(req);
    }
}

bool BinderLocationBackendHidl::gnssSetPositionMode(
    HybrisGnssPositionMode mode,
    HybrisGnssPositionRecurrence recurrence,
    uint32_t minIntervalMs,
    uint32_t preferredAccuracyMeters,
    uint32_t preferredTimeMs)
{
    bool ret = false;

    if (m_clientGnss) {
        int status = 0;
        GBinderLocalRequest *req;
        GBinderRemoteReply *reply = NULL;
        GBinderWriter writer;

        req = gbinder_client_new_request(m_clientGnss);
        gbinder_local_request_init_writer(req, &writer);
        gbinder_writer_append_int32(&writer, mode);
        gbinder_writer_append_int32(&writer, recurrence);
        gbinder_writer_append_int32(&writer, minIntervalMs);
        gbinder_writer_append_int32(&writer, preferredAccuracyMeters);
        gbinder_writer_append_int32(&writer, preferredTimeMs);
        reply = gbinder_client_transact_sync_reply(m_clientGnss,
            GNSS_SET_POSITION_MODE, req, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }

        if (!ret) {
            qWarning("GNSS set position mode failed");
        }
        gbinder_local_request_unref(req);
        gbinder_remote_reply_unref(reply);
    }
    return ret;
}

// GnssDebug
void BinderLocationBackendHidl::gnssDebugInit()
{
    GBinderRemoteReply *reply;
    int status = 0;

    reply = gbinder_client_transact_sync_reply(m_clientGnss,
        GNSS_GET_EXTENSION_GNSS_DEBUG, Q_NULLPTR, &status);

    if (!status) {
        m_remoteGnssDebug = getExtensionObject(reply);
        if (m_remoteGnssDebug) {
            qWarning("Initialising GNSS Debug interface");
            m_clientGnssDebug = gbinder_client_new(m_remoteGnssDebug, GNSS_DEBUG_REMOTE);
        }
    }
    gbinder_remote_reply_unref(reply);
}

// GnnNi
void BinderLocationBackendHidl::gnssNiInit()
{
    GBinderRemoteReply *reply;
    int status = 0;

    reply = gbinder_client_transact_sync_reply(m_clientGnss,
        GNSS_GET_EXTENSION_GNSS_NI, Q_NULLPTR, &status);

    if (!status) {
        m_remoteGnssNi = getExtensionObject(reply);

        if (m_remoteGnssNi) {
            qWarning("Initialising GNSS NI interface");
            GBinderLocalRequest *req;
            m_clientGnssNi = gbinder_client_new(m_remoteGnssNi, GNSS_NI_REMOTE);
            m_callbackGnssNi = gbinder_servicemanager_new_local_object
                (m_sm, GNSS_NI_CALLBACK, geoclue_binder_gnss_ni_callback, this);

            gbinder_remote_reply_unref(reply);

            /* IGnssNi::setCallback */
            req = gbinder_client_new_request(m_clientGnssNi);
            gbinder_local_request_append_local_object(req, m_callbackGnssNi);
            reply = gbinder_client_transact_sync_reply(m_clientGnssNi,
                GNSS_NI_SET_CALLBACK, req, &status);

            if (!status) {
                if (!gbinder_remote_reply_read_int32(reply, &status) || status != 0) {
                    qWarning("Initialising GNSS NI interface failed %d", status);
                }
            }
            gbinder_local_request_unref(req);
        }
    }
    gbinder_remote_reply_unref(reply);
}

void BinderLocationBackendHidl::gnssNiRespond(
    int32_t notifId,
    HybrisGnssUserResponseType userResponse)
{
    if (m_clientGnssNi) {
        int status = 0;
        GBinderLocalRequest *req;
        GBinderRemoteReply *reply;
        GBinderWriter writer;

        req = gbinder_client_new_request(m_clientGnssNi);
        gbinder_local_request_init_writer(req, &writer);
        gbinder_writer_append_int32(&writer, notifId);
        gbinder_writer_append_int32(&writer, userResponse);

        reply = gbinder_client_transact_sync_reply(m_clientGnssNi,
            GNSS_NI_RESPOND, req, &status);

        if (!status) {
            if (!gbinder_remote_reply_read_int32(reply, &status) || status != 0) {
                qWarning("GNSS NI respond failed %d", status);
            }
        }

        gbinder_local_request_unref(req);
        gbinder_remote_reply_unref(reply);
    }
}

// GnssXtra
void BinderLocationBackendHidl::gnssXtraInit()
{
    GBinderRemoteReply *reply;
    int status = 0;

    reply = gbinder_client_transact_sync_reply(m_clientGnss,
        GNSS_GET_EXTENSION_XTRA, Q_NULLPTR, &status);

    if (!status) {
        m_remoteGnssXtra = getExtensionObject(reply);

        if (m_remoteGnssXtra) {
            qWarning("Initialising GNSS Xtra interface");
            GBinderLocalRequest *req;
            m_clientGnssXtra = gbinder_client_new(m_remoteGnssXtra, GNSS_XTRA_REMOTE);
            m_callbackGnssXtra = gbinder_servicemanager_new_local_object
                (m_sm, GNSS_XTRA_CALLBACK, geoclue_binder_gnss_xtra_callback, this);

            gbinder_remote_reply_unref(reply);

            /* IGnssXtra::setCallback */
            req = gbinder_client_new_request(m_clientGnssXtra);
            gbinder_local_request_append_local_object(req, m_callbackGnssXtra);
            reply = gbinder_client_transact_sync_reply(m_clientGnssXtra,
                GNSS_XTRA_SET_CALLBACK, req, &status);

            if (status || !isReplySuccess(reply)) {
                qWarning("Initialising GNSS Xtra interface failed");
            }
            gbinder_local_request_unref(req);
        }
    }
    gbinder_remote_reply_unref(reply);
}

bool BinderLocationBackendHidl::gnssXtraInjectXtraData(QByteArray &xtraData)
{
    bool ret = false;
    if (m_clientGnssXtra) {
        int status = 0;

        GBinderLocalRequest *req;
        GBinderRemoteReply *reply;

        req = gbinder_client_new_request(m_clientGnssXtra);
        gbinder_local_request_append_hidl_string(req, xtraData.constData());
        reply = gbinder_client_transact_sync_reply(m_clientGnssXtra,
            GNSS_XTRA_INJECT_XTRA_DATA, req, &status);

        if (!status && isReplySuccess(reply)) {
            ret = true;
        }

        if (!ret) {
            qWarning("GNSS Xtra inject xtra data failed");
        }
        gbinder_local_request_unref(req);
        gbinder_remote_reply_unref(reply);
    }
    return ret;
}

// AGnss
void BinderLocationBackendHidl::aGnssInit()
{
    GBinderRemoteReply *reply;
    int status = 0;

    reply = gbinder_client_transact_sync_reply(m_clientGnss,
        GNSS_GET_EXTENSION_AGNSS, Q_NULLPTR, &status);

    if (!status) {
        m_remoteAGnss = getExtensionObject(reply);

        if (m_remoteAGnss) {
            qWarning("Initialising AGNSS interface");
            GBinderLocalRequest *req;
            m_clientAGnss = gbinder_client_new(m_remoteAGnss, AGNSS_REMOTE);
            m_callbackAGnss = gbinder_servicemanager_new_local_object
                (m_sm, AGNSS_CALLBACK, geoclue_binder_agnss_callback, this);

            gbinder_remote_reply_unref(reply);

            /* IAGnss::setCallback */
            req = gbinder_client_new_request(m_clientAGnss);
            gbinder_local_request_append_local_object(req, m_callbackAGnss);
            reply = gbinder_client_transact_sync_reply(m_clientAGnss,
                AGNSS_SET_CALLBACK, req, &status);

            if (!status) {
                if (!gbinder_remote_reply_read_int32(reply, &status) || status != 0) {
                    qWarning("Initialising AGNSS interface failed %d", status);
                }
            }
            gbinder_local_request_unref(req);
        }
    }
    gbinder_remote_reply_unref(reply);
}

bool BinderLocationBackendHidl::aGnssDataConnClosed()
{
    int status = 0;
    bool ret = false;
    GBinderRemoteReply *reply;

    if (!m_clientAGnss) {
        return ret;
    }

    reply = gbinder_client_transact_sync_reply(m_clientAGnss,
        AGNSS_DATA_CONN_CLOSED, Q_NULLPTR, &status);

    if (!status && isReplySuccess(reply)) {
        ret = true;
    }

    if (!ret) {
        qWarning("AGNSS data connection closed failed");
    }
    gbinder_remote_reply_unref(reply);
    return ret;
}

bool BinderLocationBackendHidl::aGnssDataConnFailed()
{
    int status = 0;
    bool ret = false;
    GBinderRemoteReply *reply;

    if (!m_clientAGnss) {
        return ret;
    }

    reply = gbinder_client_transact_sync_reply(m_clientAGnss,
        AGNSS_DATA_CONN_FAILED, Q_NULLPTR, &status);

    if (!status && isReplySuccess(reply)) {
        ret = true;
    }

    if (!ret) {
        qWarning("AGNSS data connection failed");
    }
    gbinder_remote_reply_unref(reply);
    return ret;
}

bool BinderLocationBackendHidl::aGnssDataConnOpen(
    const QByteArray &apn,
    const QString &protocol)
{
    int status = 0;
    bool ret = false;
    GBinderLocalRequest *req;
    GBinderRemoteReply *reply;
    GBinderWriter writer;

    if (!m_clientAGnss) {
        return ret;
    }

    req = gbinder_client_new_request(m_clientAGnss);

    gbinder_local_request_init_writer(req, &writer);
    gbinder_writer_append_hidl_string(&writer, apn.constData());
    gbinder_writer_append_int32(&writer, fromContextProtocol(protocol));
    reply = gbinder_client_transact_sync_reply(m_clientAGnss,
        AGNSS_DATA_CONN_OPEN, req, &status);

    if (!status && isReplySuccess(reply)) {
        ret = true;
    }

    if (!ret) {
        qWarning("AGNSS data connection open failed");
    }
    gbinder_local_request_unref(req);
    gbinder_remote_reply_unref(reply);

    return ret;
}

int BinderLocationBackendHidl::aGnssSetServer(
    HybrisAGnssType type,
    const char* hostname,
    int port)
{
    int status = 0;
    bool ret = false;
    GBinderLocalRequest *req;
    GBinderRemoteReply *reply;
    GBinderWriter writer;

    if (!m_clientAGnss) {
        return ret;
    }

    req = gbinder_client_new_request(m_clientAGnss);

    gbinder_local_request_init_writer(req, &writer);
    gbinder_writer_append_int32(&writer, type);
    gbinder_writer_append_hidl_string(&writer, hostname);
    gbinder_writer_append_int32(&writer, port);
    reply = gbinder_client_transact_sync_reply(m_clientAGnss,
        AGNSS_SET_SERVER, req, &status);

    if (!status && isReplySuccess(reply)) {
        ret = true;
    }

    gbinder_local_request_unref(req);
    gbinder_remote_reply_unref(reply);

    return ret;
}

// AGnssRil
void BinderLocationBackendHidl::aGnssRilInit()
{
    GBinderRemoteReply *reply;
    int status = 0;

    reply = gbinder_client_transact_sync_reply(m_clientGnss,
        GNSS_GET_EXTENSION_AGNSS_RIL, Q_NULLPTR, &status);

    if (!status) {
        m_remoteAGnssRil = getExtensionObject(reply);

        if (m_remoteAGnssRil) {
            qWarning("Initialising AGNSS RIL interface");
            GBinderLocalRequest *req;
            m_clientAGnssRil = gbinder_client_new(m_remoteAGnssRil, AGNSS_RIL_REMOTE);
            m_callbackAGnssRil = gbinder_servicemanager_new_local_object
                (m_sm, AGNSS_RIL_CALLBACK, geoclue_binder_agnss_ril_callback, this);

            gbinder_remote_reply_unref(reply);

            /* IAGnssRil::setCallback */
            req = gbinder_client_new_request(m_clientAGnssRil);
            gbinder_local_request_append_local_object(req, m_callbackAGnssRil);
            reply = gbinder_client_transact_sync_reply(m_clientAGnssRil,
                AGNSS_RIL_SET_CALLBACK, req, &status);

            if (!status) {
                if (!gbinder_remote_reply_read_int32(reply, &status) || status != 0) {
                    qWarning("Initialising AGNSS RIL interface failed %d", status);
                }
            }
            gbinder_local_request_unref(req);
        }
    }
    gbinder_remote_reply_unref(reply);
}
