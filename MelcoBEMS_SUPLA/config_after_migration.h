#ifndef CONFIG_AFTER_MIGRATION_H
#define CONFIG_AFTER_MIGRATION_H

#include <supla/storage/littlefs_config.h>

// Import the confirmed 1.0.24 credentials once, then use normal SUPLA storage.
class ConfigAfterMigration : public Supla::LittleFsConfig {
 public:
  explicit ConfigAfterMigration(int configSize) : LittleFsConfig(configSize) {}
  bool init() override;

 private:
  bool verifyStoredConfig();
};

#endif
