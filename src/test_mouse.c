#include <ncurses.h>

int main() {
    MEVENT event;

    initscr();              // Start curses mode
    cbreak();               // Line buffering disabled
    noecho();               // Don't echo pressed keys
    keypad(stdscr, TRUE);   // Enable function keys and arrow keys
    mousemask(ALL_MOUSE_EVENTS | REPORT_MOUSE_POSITION, NULL);

    // Enable mouse movement reporting (may be needed for some terminals)
    printf("\033[?1003h\n");  // Enable mouse move events (Xterm style)

    printw("Move mouse inside terminal window. Press 'q' to quit.\n");
    refresh();

    while (1) {
        int ch = getch();
        if (ch == 'q')
            break;

        if (ch == KEY_MOUSE) {
            if (getmouse(&event) == OK) {
                if (event.bstate & REPORT_MOUSE_POSITION) {
                    mvprintw(2, 0, "Mouse moved to: %d,%d    ", event.x, event.y);
                    refresh();
                }
            }
        }
    }

    // Disable mouse move events before exiting
    printf("\033[?1003l\n");

    endwin();   // End curses mode
    return 0;
}
