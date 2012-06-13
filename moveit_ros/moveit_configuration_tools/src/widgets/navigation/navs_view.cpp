#include "navs_view.h"

NavsView::NavsView(QWidget *parent) :
    QListView(parent)
{
    setItemDelegate(new NavDelegate(this));
    setEditTriggers(QAbstractItemView::NoEditTriggers);
    setFrameShape(QFrame::NoFrame);
    setAttribute(Qt::WA_MacShowFocusRect, false);
    verticalScrollBar()->setPageStep(3);
    verticalScrollBar()->setSingleStep(1);

    _model = new QStandardItemModel(this);
    setModel(_model);
}

void NavsView::setNavs(QList<NavItem> &navs)
{
    setModel(NULL);
    _model->clear();

    for(int i = 0; i < navs.size(); i ++)
    {
        QStandardItem *item = new QStandardItem();
        item->setData(QVariant::fromValue(navs.at(i)), Qt::DisplayRole);
        _model->appendRow(item);
    }

    setModel(_model);
}
