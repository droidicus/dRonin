/**
 ******************************************************************************
 *
 * @file       usagestatsplugin.h
 * @author     dRonin, http://dRonin.org/, Copyright (C) 2015
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup UsageStatsGadgetPlugin UsageStats Gadget Plugin
 * @{
 * @brief [Brief]
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef USAGESTATSPLUGIN_H_
#define USAGESTATSPLUGIN_H_

#include <coreplugin/iconfigurableplugin.h>
#include <coreplugin/icorelistener.h>
#include <extensionsystem/iplugin.h>
#include "uploader/uploader_global.h"
#include "uavobjectutil/devicedescriptorstruct.h"
#include <QDateTime>
#include <QAbstractButton>
#include <QNetworkReply>
#include <QNetworkAccessManager>
#include <QEventLoop>
#include <QUuid>

using namespace uploader;
struct boardLog
{
    QDateTime time;
    deviceInfo board;
    deviceDescriptorStruct device;
};
enum widgetType {WIDGET_BUTTON, WIDGET_SLIDER, WIDGET_TAB};
typedef struct widgetActionInfoType {
    widgetType type;
    QDateTime time;
    QString objectName;
    QString className;
    QString parentName;
    QString data1;
    QString data2;
} widgetActionInfo;

class UsageStatsPlugin :  public Core::IConfigurablePlugin {
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "dRonin.plugins.UsageStats" FILE "UsageStats.json")

public:
    UsageStatsPlugin();
    ~UsageStatsPlugin();
    void readConfig(QSettings *qSettings, Core::UAVConfigInfo *configInfo);
    void saveConfig(QSettings *qSettings, Core::UAVConfigInfo *configInfo);
    void shutdown();
    bool initialize(const QStringList &arguments, QString *errorString);
    void extensionsInitialized();
    bool coreAboutToClose();
    bool getSendUsageStats() const;
    void setSendUsageStats(bool value);

    bool getSendPrivateData() const;
    void setSendPrivateData(bool value);

    QString getInstallationUUID() const;
public slots:
    void updateSettings();
private:
    ExtensionSystem::PluginManager *pluginManager;
    QList<boardLog> boardLogList;
    QList<widgetActionInfo> widgetLogList;
    QByteArray processJson();
    QString externalIP;
    QNetworkAccessManager netMngr;
    QEventLoop *loop;
    bool sendUsageStats;
    bool sendPrivateData;
    QUuid installationUUID;
private slots:
    void pluginsLoadEnded();
    void addNewBoardSeen(deviceInfo, deviceDescriptorStruct);
    void searchForWidgets(QObject *mw, bool connect);
    void onButtonClicked();
    void onSliderValueChanged(int);
    void onTabCurrentChanged(int);
};
class AppCloseHook : public Core::ICoreListener {
    Q_OBJECT
public:
    AppCloseHook(UsageStatsPlugin *parent);
    bool coreAboutToClose();
private:
    UsageStatsPlugin *m_parent;
};
#endif /* USAGESTATSPLUGIN_H_ */
