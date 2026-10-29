#include "ComputerLocation.h"
#include <QGeoCoordinate>
#ifdef Q_OS_WIN
#include <windows.h>
#include <initguid.h>
#include <locationapi.h>
#endif

ComputerLocation::ComputerLocation(QObject *parent) : QObject(parent) {
    m_timer.setInterval(1000);
    connect(&m_timer, &QTimer::timeout, this, &ComputerLocation::poll);
}

ComputerLocation::~ComputerLocation() { release(); }

void ComputerLocation::request() {
#ifdef Q_OS_WIN
    if (m_location) return;
    const auto initialized = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    if (FAILED(initialized) && initialized != RPC_E_CHANGED_MODE) return;
    m_ownsCom = SUCCEEDED(initialized);
    ILocation *location = nullptr;
    if (FAILED(CoCreateInstance(CLSID_Location, nullptr, CLSCTX_INPROC_SERVER,
                                IID_ILocation, reinterpret_cast<void **>(&location)))) {
        release();
        return;
    }
    m_location = location;
    IID reports[] = { IID_ILatLongReport };
    // Honor the OS permission decision; never change location/privacy settings.
    if (FAILED(location->RequestPermissions(nullptr, reports, 1, FALSE))) {
        release();
        return;
    }
    m_attempts = 0;
    m_timer.start();
    poll();
#endif
}

void ComputerLocation::poll() {
#ifdef Q_OS_WIN
    if (!m_location) return;
    ILocationReport *report = nullptr;
    if (SUCCEEDED(static_cast<ILocation *>(m_location)->GetReport(IID_ILatLongReport, &report))) {
        ILatLongReport *coordinates = nullptr;
        if (SUCCEEDED(report->QueryInterface(IID_ILatLongReport, reinterpret_cast<void **>(&coordinates)))) {
            double latitude = 0, longitude = 0;
            const bool valid = SUCCEEDED(coordinates->GetLatitude(&latitude))
                && SUCCEEDED(coordinates->GetLongitude(&longitude))
                && QGeoCoordinate(latitude, longitude).isValid();
            coordinates->Release();
            report->Release();
            if (valid) {
                release();
                emit located(latitude, longitude);
                return;
            }
        } else report->Release();
    }
    if (++m_attempts >= 15) release();
#endif
}

void ComputerLocation::release() {
    m_timer.stop();
#ifdef Q_OS_WIN
    if (m_location) static_cast<ILocation *>(m_location)->Release();
    m_location = nullptr;
    if (m_ownsCom) CoUninitialize();
    m_ownsCom = false;
#endif
}
