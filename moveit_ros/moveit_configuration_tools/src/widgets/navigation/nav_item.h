#ifndef MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_NAVIGATION_NAVITEM_
#define MOVEIT_ROS_MOVEIT_CONFIGURATION_TOOLS_WIDGETS_NAVIGATION_NAVITEM_

#include <QString>
#include <QMetaType>

class NavItem
{
public:
    explicit NavItem();
    NavItem(const QString &firstname);
    virtual ~NavItem() { ; }

    QString name() const;
private:
    long _id;

    QString _firstname;
};

Q_DECLARE_METATYPE(NavItem);

#endif // PERSON_H
