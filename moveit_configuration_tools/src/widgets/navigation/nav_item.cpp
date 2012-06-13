#include "nav_item.h"

NavItem::NavItem()
{

}

NavItem::NavItem(const QString &firstname) :
    _firstname(firstname)
{

}

QString NavItem::name() const
{
  return _firstname;
}
