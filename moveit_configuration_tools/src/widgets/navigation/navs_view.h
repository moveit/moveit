#ifndef PEOPLEVIEW_H
#define PEOPLEVIEW_H

#include <QListView>
#include <QStandardItemModel>
#include <QScrollBar>

#include "nav_item.h"
#include "nav_delegate.h"

class NavsView : public QListView
{
    Q_OBJECT
public:
    explicit NavsView(QWidget *parent = 0);

    void setNavs(QList<NavItem> &people);

private:
    QStandardItemModel *_model;
};

#endif // PEOPLEVIEW_H
