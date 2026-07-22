#pragma once
namespace Communications {
typedef struct struct_message {
    bool triggerState;
} struct_message;
bool init(void);
bool send_message(struct_message message);
}  // namespace Communications
