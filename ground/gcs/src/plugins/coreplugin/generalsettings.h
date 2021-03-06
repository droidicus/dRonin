/**
 ******************************************************************************
 *
 * @file       generalsettings.h
 * @author     dRonin, http://dronin.org Copyright (C) 2015
 * @author     Tau Labs, http://taulabs.org, Copyright (C) 2014
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2010.
 *             Parts by Nokia Corporation (qt-info@nokia.com) Copyright (C) 2009.
 * @addtogroup GCSPlugins GCS Plugins
 * @{
 * @addtogroup CorePlugin Core Plugin
 * @{
 * @brief The Core GCS plugin
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

#ifndef GENERALSETTINGS_H
#define GENERALSETTINGS_H

#include <coreplugin/dialogs/ioptionspage.h>
#include <QtCore/QPointer>
#include <QWidget>
#include <QSettings>
#include <QtNetwork/QNetworkProxy>

namespace Core {
namespace Internal {

namespace Ui {
    class GeneralSettings;
}

class CORE_EXPORT GeneralSettings : public IOptionsPage
{
    Q_OBJECT

public:
    GeneralSettings();

    QString id() const;
    QString trName() const;
    QString category() const;
    QString trCategory() const;
    QWidget* createPage(QWidget *parent);
    void apply();
    void finish();
    bool saveSettingsOnExit() const;
    bool autoConnect() const;
    bool autoSelect() const;
    bool useUDPMirror() const;
    bool useSessionManaging() const;
    void readSettings(QSettings* qs);
    void saveSettings(QSettings* qs);
    bool useExpertMode() const;
    QNetworkProxy getNetworkProxy();
    void setObservations(QString value);
    QString getObservations();
    void setVehicleType(QString type);
    QString getVehicleType();
    void setBoardType(QString type);
    QString getBoardType();
    void setWeight(int weight);
    int getWeight();
    void setVehicleSize(int spacing);
    int getVehicleSize();
    void setBatteryCells(int cells);
    int getBatteryCells();
    void setMotors(QString motors);
    QString getMotors();
    void setESCs(QString escs);
    QString getESCs();
    void setProps(QString props);
    QString getProps();
signals:
    void generalSettingsChanged();
private slots:
    void resetInterfaceColor();
    void resetLanguage();
    void showHelpForExternalEditor();
    void slotAutoConnect(int);    

private:
    void fillLanguageBox() const;
    void fillProxyTypesBox() const;
    QString language() const;
    void setLanguage(const QString&);
    Ui::GeneralSettings *m_page;
    QString m_language;
    bool m_saveSettingsOnExit;
    bool m_autoConnect;
    bool m_autoSelect;
    bool m_useUDPMirror;
    bool m_useExpertMode;
    QPointer<QWidget> m_dialog;
    QList<QTextCodec *> m_codecs;
    int m_proxyType;
    int m_proxyPort;
    QString m_proxyHostname;
    QString m_proxyUser;
    QString m_proxyPassword;
    QString m_observations;
    QString m_vehicle;
    QString m_board;
    int m_weight;
    int m_size;
    int m_cells;
    QString m_motors;
    QString m_escs;
    QString m_props;
    bool m_useSessionManaging;
};
} // namespace Internal
} // namespace Core

#endif // GENERALSETTINGS_H
