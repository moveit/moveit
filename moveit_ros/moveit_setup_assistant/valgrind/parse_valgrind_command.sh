#http://wiki.wxwidgets.org/Valgrind_Suppression_File_Howto
#rm -f qt_valgrind_min.log
#rm -f qt_valgrind_min.supp
valgrind --leak-check=full --show-reachable=yes --error-limit=no --gen-suppressions=all --log-file=qt_valgrind_min.log ../bin/start_screen_widget
cat ./qt_valgrind_min.log | ./parse_valgrind_suppressions.sh > qt_valgrind_min.supp
