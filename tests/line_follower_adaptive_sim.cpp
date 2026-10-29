#include <bits/stdc++.h>
using namespace std;

/*
 * Desktop-only logic simulator for line_follower_adaptive_v2.c.
 * This file is NOT uploaded to the Huna controller. <bits/stdc++.h> is a
 * GNU C++ convenience header and may not exist in the Huna firmware toolchain.
 *
 * Input patterns are ordered left-center-right; 1 means black detected.
 * Output is a qualitative motor command, not a physical robot simulation.
 */
struct Command {
    int left;
    int right;
    string state;
};

Command decide(bool l, bool c, bool r, int lastTurn) {
    constexpr int STRAIGHT = 5;
    constexpr int MEDIUM = 4;
    constexpr int TIGHT = 2;

    if (l && c && r) return {0, 0, "crossing"};
    if (l && c)       return {0, MEDIUM, "curve-left-medium"};
    if (c && r)       return {MEDIUM, 0, "curve-right-medium"};
    if (l)            return {0, TIGHT, "sharp-left-slow"};
    if (r)            return {TIGHT, 0, "sharp-right-slow"};
    if (c)            return {STRAIGHT, STRAIGHT, "straight-fast"};
    if (lastTurn < 0) return {0, TIGHT, "line-lost-search-left"};
    if (lastTurn > 0) return {TIGHT, 0, "line-lost-search-right"};
    return {0, 0, "line-lost-stop"};
}

int main() {
    const vector<tuple<bool,bool,bool,string>> cases = {
        {false,true,false,"center only"},
        {true,true,false,"left + center"},
        {false,true,true,"center + right"},
        {true,false,false,"left only"},
        {false,false,true,"right only"},
        {true,true,true,"all black / crossing"},
        {false,false,false,"no sensor sees line"}
    };

    cout << "Pattern\tLeftMotor\tRightMotor\tDecision\n";
    for (const auto &[l,c,r,label] : cases) {
        const auto out = decide(l,c,r,0);
        cout << label << '\t' << out.left << '\t' << out.right
             << '\t' << out.state << '\n';
    }
    return 0;
}
