#pragma once
namespace Communications {

typedef struct cameraTrigger {
    bool triggerState;
} cameraTrigger;

typedef struct detectionResult {
    bool teletubbyFound;
    int teletubbyCount;
} detectionResult;

bool init(void);
bool send_message(cameraTrigger message);
bool hasNewMessage(void);
void resetHasNewMessage(void);
bool teletubbyFound(void);
int teletubbyCount(void);
}  // namespace Communications
