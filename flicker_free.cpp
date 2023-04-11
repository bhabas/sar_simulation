#include <ncurses.h>
#include <unistd.h>
#include <ctime>

int main() {
    // Initialize the ncurses library
    initscr();
    timeout(0); // Set getch() to non-blocking mode
    curs_set(0); // Hide the cursor

    int count = 0;
    const int refresh_rate = 50; // 50 Hz
    const int delay_time_us = 1000000 / refresh_rate;

    while (true) {
        // Clear the screen buffer
        clear();

        // Print data to the screen buffer
        mvprintw(0, 0, "Count: %d", count++);

        // Refresh the screen with the updated buffer
        refresh();

        // Check for user input to exit the program
        int ch = getch();
        if (ch == 'q' || ch == 'Q') {
            break;
        }

        // Sleep for the desired delay time
        usleep(delay_time_us);
    }

    // Clean up and close the ncurses library
    endwin();

    return 0;
}
