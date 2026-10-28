# 无人机空投物资投递

按投递点列表依次飞抵、悬停锁定、触发放物, 最后返航。

## 流程

```mermaid
flowchart TD
    A[飞抵投递点] --> B[位置锁定悬停]
    B --> C[发布 drop_command]
    C --> D{还有点?}
    D -->|是| A
    D -->|否| E[返航]
```

## 运行

```bash
cd src/air/uav_parcel_delivery
python3 main.py
```

发布 `/drone/setpoint_position`、`/drone/drop_command`、`/delivery_status`。详见模块 `README.md`。
