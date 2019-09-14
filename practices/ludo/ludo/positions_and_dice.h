#ifndef POSITIONS_AND_DICE_H
#define POSITIONS_AND_DICE_H
#include <vector>

struct positions_and_dice {
    std::vector<int> pos;
    int dice;
    positions_and_dice() {}
    positions_and_dice(int v) : dice(v) {}
};
#endif // POSITIONS_AND_DICE_H
