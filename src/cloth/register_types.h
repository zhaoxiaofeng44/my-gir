#ifndef CLOTH_NATIVE_REGISTER_TYPES_H
#define CLOTH_NATIVE_REGISTER_TYPES_H

#include <godot_cpp/core/class_db.hpp>

using namespace godot;

void initialize_cloth_native_module(ModuleInitializationLevel p_level);
void uninitialize_cloth_native_module(ModuleInitializationLevel p_level);

#endif
