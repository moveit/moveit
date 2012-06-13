#ifndef PERSONDELEGATE_H
#define PERSONDELEGATE_H

#include <QStyledItemDelegate>
#include <QPainter>

#include "nav_item.h"

class NavDelegate : public QStyledItemDelegate
{
    Q_OBJECT
public:
    explicit NavDelegate(QObject *parent = 0);

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const;

private:
    void paintNav(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;
    void paintLetter(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const;

};

#endif // PERSONDELEGATE_H
