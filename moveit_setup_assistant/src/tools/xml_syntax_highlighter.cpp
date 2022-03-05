/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Bielefeld University, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Bielefeld University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Robert Haschke */

#include "xml_syntax_highlighter.h"

XmlSyntaxHighlighter::XmlSyntaxHighlighter(QTextDocument* parent) : QSyntaxHighlighter(parent)
{
}

void XmlSyntaxHighlighter::addTag(const QString& tag, const QTextCharFormat& format, const QString& parent)
{
  const QString start_pattern("<%1.*?/?>");
  Rule rule;
  rule.start = QRegularExpression(start_pattern.arg(tag));
  rule.end = QRegularExpression(QString("</%1>|<%1[^>]*?/>").arg(tag));
  rule.format = format;
  if (!parent.isEmpty())
  {
    QString parent_start = start_pattern.arg(parent);
    rule.parent = std::find_if(rules.begin(), rules.end(), [&](const std::pair<int, Rule>& rule) {
      return rule.second.start.pattern() == parent_start;
    });
  }
  else
    rule.parent = rules.end();

  rules.insert(std::make_pair(rules.size(), rule));
}

void XmlSyntaxHighlighter::highlightBlock(const QString& text)
{
  int active_rule = previousBlockState();
  int start;
  if (active_rule < 0)  // no rule active from previous block
  {
    start = text.size();
    for (const auto& r : rules)  // process all top-level rules
    {
      const auto& rule = r.second;
      if (rule.parent != rules.end())
        continue;

      int pos = text.indexOf(rule.start);
      if (pos >= 0 && pos < start)
      {
        start = pos;
        active_rule = r.first;
      }
    }
    if (start == text.size())
      return;  // found nothing to highlight
  }
  else
    start = 0;

  while (start >= 0)
  {
    const auto& rule = rules[active_rule];
    auto match = rule.end.match(text, start);
    if (!match.hasMatch())
      setCurrentBlockState(active_rule);
    setFormat(start, (match.hasMatch() ? match.capturedEnd() : text.size()) - start, rule.format);
    start = -1;
  }
}
