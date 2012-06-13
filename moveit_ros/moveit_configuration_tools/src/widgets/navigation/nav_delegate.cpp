#include "nav_delegate.h"

NavDelegate::NavDelegate(QObject *parent) :
    QStyledItemDelegate(parent)
{
}

void NavDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    if(index.data().canConvert<NavItem>())
        paintNav(painter, option, index);
    else
        paintLetter(painter, option, index);
}
QSize NavDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    if(index.data().canConvert<NavItem>())
        return QSize(option.rect.width(), 45);
    else
        return QSize(option.rect.width(), 25);
}

void NavDelegate::paintNav(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    const bool isSelected = option.state & QStyle::State_Selected;
    const bool isHovered = option.state & QStyle::State_MouseOver;

    NavItem tp = index.data().value<NavItem>();

    painter->save();

    QLinearGradient backgroundGradient(QPoint(option.rect.x(), option.rect.y()), QPoint(option.rect.x(), option.rect.y()+option.rect.height()));
    if(isSelected) {
        //        painter->fillRect(option.rect, QBrush(QColor(49, 49, 49)));
        backgroundGradient.setColorAt(0, QColor(109, 164, 219));
        backgroundGradient.setColorAt(1, QColor(61, 138, 212));
        painter->fillRect(option.rect, QBrush(backgroundGradient));
        //        painter->fillRect(option.rect, QBrush(QColor(225, 225, 225)));

    } else {
        //        painter->fillRect(option.rect, QBrush(QColor(244, 244, 244)));
        backgroundGradient.setColorAt(0, QColor(245, 245, 245));
        backgroundGradient.setColorAt(1, QColor(240, 240, 240));
        painter->fillRect(option.rect, QBrush(backgroundGradient));
    }

    painter->setPen(QColor(225, 225, 225));
    if(isSelected)
    {
        painter->setPen(QColor(37, 105, 169));
        painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight());
        painter->setPen(Qt::transparent);
    }
    painter->drawLine(option.rect.topLeft(), option.rect.topRight());
    if(!isSelected)
    {
        painter->setPen(QColor(248, 248, 248));
        painter->drawLine(QPoint(option.rect.x(), option.rect.y()+1), QPoint(option.rect.x()+option.rect.width(), option.rect.y()+1));
    }
    //    QString text = index.model()->data(index, Qt::DisplayRole).toString();
    QRect textRect(option.rect.x()+10, option.rect.y(), option.rect.width()-10, option.rect.height());
    painter->setPen(QColor(69, 69, 69));
    QFont textFont(painter->font());
    textFont.setPixelSize(18);
    if(isSelected){
        painter->setPen(QColor(229, 229, 229));
    }

    textFont.setFamily("Helvetica Neue");
    painter->setFont(textFont);
    painter->drawText(textRect, Qt::AlignLeft|Qt::AlignVCenter, tp.name());

    painter->restore();
}
void NavDelegate::paintLetter(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    const bool isSelected = option.state & QStyle::State_Selected;
    const bool isHovered = option.state & QStyle::State_MouseOver;

    painter->save();

    QLinearGradient backgroundGradient(QPoint(option.rect.x(), option.rect.y()), QPoint(option.rect.x(), option.rect.y()+option.rect.height()));
    //        painter->fillRect(option.rect, QBrush(QColor(225, 225, 225)));

    backgroundGradient.setColorAt(0, QColor(215, 215, 215));
    backgroundGradient.setColorAt(1, QColor(230, 230, 230));
    painter->fillRect(option.rect, QBrush(backgroundGradient));

    painter->setPen(QColor(213, 213, 213));
    painter->drawLine(option.rect.bottomLeft(), option.rect.bottomRight());

    QFont textFont(painter->font());
    textFont.setPixelSize(18);
    textFont.setFamily("Helvetica Neue");
    painter->setFont(textFont);

    QRect textRect(option.rect.x()+10, option.rect.y(), option.rect.width()-10, option.rect.height());
    painter->setPen(QColor(39, 39, 39));
    painter->drawText(textRect, Qt::AlignLeft|Qt::AlignVCenter, index.model()->data(index).toString());

    painter->restore();
}

C++/l finished at Tue Jun 12 19:13:37
