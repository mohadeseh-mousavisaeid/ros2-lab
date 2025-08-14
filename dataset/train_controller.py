import torch
import torch.nn as nn
from torch.utils.data import DataLoader, TensorDataset
import pandas as pd

# --- Load CSV ---
df = pd.read_csv('/home/projectlab3_ss25/our_ws/dataset/all_data_merged_2025-08-13_14-17-41.csv')

# --- Fill missing values ---
df = df.fillna(0.0)

# --- Define Inputs and Outputs ---
X = df[[
    'odom_x', 'odom_y', 'odom_vx', 'odom_vy', 'odom_angular_z',
    'initialpose_x', 'initialpose_y',
    'goal_x', 'goal_y', 'goal_z',
    'goal_qx', 'goal_qy', 'goal_qz', 'goal_qw'
]].values.astype('float32')

Y = df[['cmd_vel_vx', 'cmd_vel_angular_z']].values.astype('float32')

# --- Dataset and Dataloader ---
dataset = TensorDataset(torch.tensor(X), torch.tensor(Y))
loader = DataLoader(dataset, batch_size=32, shuffle=True)

# --- Neural Network Model ---
class ControllerNN(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(14, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 2)
        )

    def forward(self, x):
        return self.net(x)

model = ControllerNN()
optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
loss_fn = nn.MSELoss()

# --- Training Loop ---
for epoch in range(100):
    total_loss = 0
    for xb, yb in loader:
        pred = model(xb)
        loss = loss_fn(pred, yb)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    print(f"Epoch {epoch}, Loss {total_loss:.4f}")

# --- Save the Model ---
torch.save(model.state_dict(), '13_08__nn_controller.pt')
print("Model trained and saved as 13_08_nn_controller.pt")


# # train_controller.py
# import torch
# import torch.nn as nn
# from torch.utils.data import DataLoader, TensorDataset
# import pandas as pd

# # --- Load CSV ---
# df = pd.read_csv('all_data_merged.csv')

# # Drop rows with NaNs
# # df = df.dropna(subset=[
# #     'odom_x', 'odom_y', 'odom_vx', 'odom_vy', 'odom_angular_z',
# #     'initialpose_x', 'initialpose_y',
# #     'cmd_vel_vx', 'cmd_vel_angular_z'
# # ])
# df = df.fillna(0.0)


# # --- Define Inputs and Outputs ---
# X = df[[
#     'odom_x', 'odom_y', 'odom_vx', 'odom_vy', 'odom_angular_z',
#     'initialpose_x', 'initialpose_y'
# ]].values.astype('float32')

# Y = df[['cmd_vel_vx', 'cmd_vel_angular_z']].values.astype('float32')

# # --- Dataset and Dataloader ---
# dataset = TensorDataset(torch.tensor(X), torch.tensor(Y))
# loader = DataLoader(dataset, batch_size=32, shuffle=True)

# # --- Neural Network Model ---
# class ControllerNN(nn.Module):
#     def __init__(self):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(7, 64),
#             nn.ReLU(),
#             nn.Linear(64, 64),
#             nn.ReLU(),
#             nn.Linear(64, 2)
#         )

#     def forward(self, x):
#         return self.net(x)

# model = ControllerNN()
# optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
# loss_fn = nn.MSELoss()

# # --- Training Loop ---
# for epoch in range(100):
#     total_loss = 0
#     for xb, yb in loader:
#         pred = model(xb)
#         loss = loss_fn(pred, yb)
#         optimizer.zero_grad()
#         loss.backward()
#         optimizer.step()
#         total_loss += loss.item()
#     print(f"Epoch {epoch}, Loss {total_loss:.4f}")

# # --- Save the Model ---
# torch.save(model.state_dict(), 'new_nn_controller.pt')
# print("✅ Model trained and saved as new_nn_controller.pt")




# # train_controller.py
# import torch
# import torch.nn as nn
# from torch.utils.data import DataLoader, TensorDataset
# import pandas as pd

# # Load CSV
# df = pd.read_csv('dataset.csv')
# X = df[['x', 'y', 'yaw']].values.astype('float32')
# Y = df[['v', 'omega']].values.astype('float32')

# # Dataset
# dataset = TensorDataset(torch.tensor(X), torch.tensor(Y))
# loader = DataLoader(dataset, batch_size=32, shuffle=True)

# # Model
# class ControllerNN(nn.Module):
#     def __init__(self):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(3, 64),
#             nn.ReLU(),
#             nn.Linear(64, 64),
#             nn.ReLU(),
#             nn.Linear(64, 2)
#         )

#     def forward(self, x):
#         return self.net(x)

# model = ControllerNN()
# optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
# loss_fn = nn.MSELoss()

# # Training loop
# for epoch in range(100):
#     total_loss = 0
#     for xb, yb in loader:
#         pred = model(xb)
#         loss = loss_fn(pred, yb)
#         optimizer.zero_grad()
#         loss.backward()
#         optimizer.step()
#         total_loss += loss.item()
#     print(f"Epoch {epoch}, Loss {total_loss:.4f}")

# # Save model
# torch.save(model.state_dict(), 'nn_controller.pt')
# # train_controller.py
# import torch
# import torch.nn as nn
# from torch.utils.data import DataLoader, TensorDataset
# import pandas as pd

# # Load CSV
# df = pd.read_csv('dataset.csv')
# X = df[['x', 'y', 'yaw']].values.astype('float32')
# Y = df[['v', 'omega']].values.astype('float32')

# # Dataset
# dataset = TensorDataset(torch.tensor(X), torch.tensor(Y))
# loader = DataLoader(dataset, batch_size=32, shuffle=True)

# # Model
# class ControllerNN(nn.Module):
#     def __init__(self):
#         super().__init__()
#         self.net = nn.Sequential(
#             nn.Linear(3, 64),
#             nn.ReLU(),
#             nn.Linear(64, 64),
#             nn.ReLU(),
#             nn.Linear(64, 2)
#         )

#     def forward(self, x):
#         return self.net(x)

# model = ControllerNN()
# optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
# loss_fn = nn.MSELoss()

# # Training loop
# for epoch in range(100):
#     total_loss = 0
#     for xb, yb in loader:
#         pred = model(xb)
#         loss = loss_fn(pred, yb)
#         optimizer.zero_grad()
#         loss.backward()
#         optimizer.step()
#         total_loss += loss.item()
#     print(f"Epoch {epoch}, Loss {total_loss:.4f}")

# # Save model
# torch.save(model.state_dict(), 'nn_controller.pt')
