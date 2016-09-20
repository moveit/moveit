#include "rotated_header_view.h"
#include <QPainter>
#include <QDebug>

RotatedHeaderView::RotatedHeaderView(Qt::Orientation orientation, QWidget *parent) : QHeaderView(orientation, parent)
{
#if (QT_VERSION >= QT_VERSION_CHECK(5, 0, 0))
  setSectionsClickable(true);
  setSectionResizeMode(Fixed);
#else
  setClickable(true);
  setResizeMode(Fixed);
#endif
  setDefaultSectionSize(27);
}

void RotatedHeaderView::paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const
{
  if (orientation() == Qt::Vertical)
  {
    QHeaderView::paintSection(painter, rect, logicalIndex);
    return;
  }

  painter->save();
  // rotate about (x,y)
  painter->translate(rect.x(), rect.y());
  painter->rotate(-90);
  painter->translate(-rect.height(), 0);
  QHeaderView::paintSection(painter, QRect(0, 0, rect.height(), rect.width()), logicalIndex);
  painter->restore();
}

QSize RotatedHeaderView::sectionSizeFromContents(int logicalIndex) const
{
  if (orientation() == Qt::Vertical)
    return QHeaderView::sectionSizeFromContents(logicalIndex);

  Q_ASSERT(logicalIndex >= 0);

  ensurePolished();

  // use SizeHintRole
  QVariant variant = model()->headerData(logicalIndex, Qt::Vertical, Qt::SizeHintRole);
  if (variant.isValid())
    return qvariant_cast<QSize>(variant);

  // otherwise use the contents
  QStyleOptionHeader opt;
  initStyleOption(&opt);
  opt.section = logicalIndex;
  QVariant var = model()->headerData(logicalIndex, orientation(), Qt::FontRole);
  QFont fnt;
  if (var.isValid() && var.canConvert<QFont>())
    fnt = qvariant_cast<QFont>(var);
  else
    fnt = font();
  fnt.setBold(true);
  opt.fontMetrics = QFontMetrics(fnt);
  opt.text = model()->headerData(logicalIndex, orientation(), Qt::DisplayRole).toString();
  variant = model()->headerData(logicalIndex, orientation(), Qt::DecorationRole);
  opt.icon = qvariant_cast<QIcon>(variant);
  if (opt.icon.isNull())
    opt.icon = qvariant_cast<QPixmap>(variant);
  QSize size = style()->sizeFromContents(QStyle::CT_HeaderSection, &opt, QSize(), this);
  if (isSortIndicatorShown())
  {
    int margin = style()->pixelMetric(QStyle::PM_HeaderMargin, &opt, this);
    if (orientation() == Qt::Horizontal)
      size.rwidth() += size.height() + margin;
    else
      size.rheight() += size.width() + margin;
  }
  return QSize(size.height(), size.width());
}

int RotatedHeaderView::sectionSizeHint(int logicalIndex) const
{
  if (isSectionHidden(logicalIndex))
    return 0;
  if (logicalIndex < 0 || logicalIndex >= count())
    return -1;
  QSize size;
  QVariant value = model()->headerData(logicalIndex, orientation(), Qt::SizeHintRole);
  if (value.isValid())
    size = qvariant_cast<QSize>(value);
  else
    size = sectionSizeFromContents(logicalIndex);
  int hint = size.height();
  qDebug() << logicalIndex << size << hint;
  return qMax(minimumSectionSize(), hint);
}
