

#include "backward.hpp"
#include "daheng_cam/daheng.hpp"
#include "param_manager.hpp"

backward::SignalHandling _;                   ///< 用来在程序崩掉时打印堆栈信息
ParamManager &pm = ParamManager::instance();  ///< 参数配置管理器

int main() {
  pm.SetConfigPath(CONFIG_DIR);
  pm.Update();
  return 0;
}