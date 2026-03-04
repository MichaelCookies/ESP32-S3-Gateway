#ifndef SMART_SCENE_ENGINE_H
#define SMART_SCENE_ENGINE_H

#ifdef __cplusplus
extern "C" {
#endif

// 启动智能场景引擎
void smart_scene_engine_start(void);

// 暴露给外界，如网络控制等的安全执行器操作接口
void safe_set_actuator(const char* node_id, bool target_state);

#ifdef __cplusplus
}
#endif

#endif // SMART_SCENE_ENGINE_H