#include <QHeaderView>
#ifndef ROTATEDHEADERVIWE_H
#define ROTATEDHEADERVIWE_H
class RotatedHeaderView : public QHeaderView
{
public:
  RotatedHeaderView(Qt::Orientation orientation, QWidget *parent = NULL);
  void paintSection(QPainter *painter, const QRect &rect, int logicalIndex) const;
  QSize sectionSizeFromContents(int logicalIndex) const;
  int sectionSizeHint(int logicalIndex) const;
};
#endif  // ROTATEDHEADERVIWE_H
