#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
意图数据分析脚本
用于分析从动态预测器导出的意图判断数据
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import rcParams

# 设置中文字体
rcParams['font.sans-serif'] = ['SimHei', 'DejaVu Sans']
rcParams['axes.unicode_minus'] = False

class IntentDataAnalyzer:
    def __init__(self, csv_file_path):
        """
        初始化分析器
        
        Args:
            csv_file_path (str): CSV文件路径
        """
        self.csv_file_path = csv_file_path
        self.data = None
        self.load_data()
    
    def load_data(self):
        """加载CSV数据"""
        try:
            # 跳过注释行，加载数据
            with open(self.csv_file_path, 'r') as f:
                lines = f.readlines()
            
            # 找到数据开始的行
            data_start = 0
            for i, line in enumerate(lines):
                if not line.startswith('#'):
                    data_start = i
                    break
            
            # 读取数据
            self.data = pd.read_csv(self.csv_file_path, skiprows=data_start, header=None)
            
            # 设置列名
            columns = [
                'Timestamp', 'Obstacle_ID', 'NumHistFrames',
                'Current_Pos_X', 'Current_Pos_Y', 'Current_Pos_Z', 'Current_Vel_X', 'Current_Vel_Y', 'Current_Vel_Z', 'Current_Acc_X', 'Current_Acc_Y', 'Current_Acc_Z',
                'Prev_Pos_X', 'Prev_Pos_Y', 'Prev_Pos_Z', 'Prev_Vel_X', 'Prev_Vel_Y', 'Prev_Vel_Z', 'Prev_Acc_X', 'Prev_Acc_Y', 'Prev_Acc_Z',
                'Oldest_Pos_X', 'Oldest_Pos_Y', 'Oldest_Pos_Z', 'Oldest_Vel_X', 'Oldest_Vel_Y', 'Oldest_Vel_Z', 'Oldest_Acc_X', 'Oldest_Acc_Y', 'Oldest_Acc_Z',
                'Size_X', 'Size_Y', 'Size_Z',
                'Intent_Forward', 'Intent_Left', 'Intent_Right', 'Intent_Stop'
            ]
            
            self.data.columns = columns
            
            print(f"成功加载数据：{len(self.data)} 行，{len(self.data.columns)} 列")
            print(f"包含 {self.data['Obstacle_ID'].nunique()} 个障碍物")
            
        except Exception as e:
            print(f"加载数据失败: {e}")
            return None
    
    def analyze_intent_distribution(self):
        """分析意图分布"""
        if self.data is None:
            return
        
        # 获取每个障碍物的最新意图概率
        latest_data = self.data.groupby('Obstacle_ID').last()
        
        # 计算意图分布
        intent_columns = ['Intent_Forward', 'Intent_Left', 'Intent_Right', 'Intent_Stop']
        intent_means = latest_data[intent_columns].mean()
        
        print("\n=== 意图分布分析 ===")
        print("平均意图概率:")
        for intent, prob in intent_means.items():
            intent_name = intent.replace('Intent_', '')
            print(f"  {intent_name}: {prob:.3f}")
        
        # 绘制意图分布图
        plt.figure(figsize=(10, 6))
        
        # 子图1: 平均意图概率
        plt.subplot(1, 2, 1)
        intent_names = ['前进', '左转', '右转', '停止']
        plt.bar(intent_names, intent_means.values)
        plt.title('平均意图概率分布')
        plt.ylabel('概率')
        plt.ylim(0, 1)
        
        # 为每个柱子添加数值标签
        for i, v in enumerate(intent_means.values):
            plt.text(i, v + 0.01, f'{v:.3f}', ha='center')
        
        # 子图2: 主导意图分布
        plt.subplot(1, 2, 2)
        dominant_intents = latest_data[intent_columns].idxmax(axis=1)
        dominant_counts = dominant_intents.value_counts()
        
        # 替换标签名称
        label_map = {
            'Intent_Forward': '前进',
            'Intent_Left': '左转', 
            'Intent_Right': '右转',
            'Intent_Stop': '停止'
        }
        dominant_counts.index = dominant_counts.index.map(label_map)
        
        plt.pie(dominant_counts.values, labels=dominant_counts.index, autopct='%1.1f%%')
        plt.title('主导意图分布')
        
        plt.tight_layout()
        plt.show()
    
    def analyze_trajectory_patterns(self):
        """分析轨迹模式"""
        if self.data is None:
            return
        
        print("\n=== 轨迹模式分析 ===")
        
        # 计算每个障碍物的轨迹特征
        trajectory_stats = []
        
        for ob_id in self.data['Obstacle_ID'].unique():
            ob_data = self.data[self.data['Obstacle_ID'] == ob_id].iloc[-1]  # 取最新的一条记录
            
            # 计算移动距离（从最旧到当前位置）
            current_pos = np.array([ob_data['Current_Pos_X'], ob_data['Current_Pos_Y'], ob_data['Current_Pos_Z']])
            prev_pos = np.array([ob_data['Prev_Pos_X'], ob_data['Prev_Pos_Y'], ob_data['Prev_Pos_Z']])
            oldest_pos = np.array([ob_data['Oldest_Pos_X'], ob_data['Oldest_Pos_Y'], ob_data['Oldest_Pos_Z']])
            
            # 计算轨迹长度
            dist_oldest_to_prev = np.linalg.norm(prev_pos - oldest_pos)
            dist_prev_to_current = np.linalg.norm(current_pos - prev_pos)
            trajectory_length = dist_oldest_to_prev + dist_prev_to_current
            
            # 计算速度
            current_vel = np.array([ob_data['Current_Vel_X'], ob_data['Current_Vel_Y'], ob_data['Current_Vel_Z']])
            prev_vel = np.array([ob_data['Prev_Vel_X'], ob_data['Prev_Vel_Y'], ob_data['Prev_Vel_Z']])
            oldest_vel = np.array([ob_data['Oldest_Vel_X'], ob_data['Oldest_Vel_Y'], ob_data['Oldest_Vel_Z']])
            
            speeds = [np.linalg.norm(current_vel), np.linalg.norm(prev_vel), np.linalg.norm(oldest_vel)]
            avg_speed = np.mean(speeds)
            max_speed = np.max(speeds)
            
            # 计算加速度
            current_acc = np.array([ob_data['Current_Acc_X'], ob_data['Current_Acc_Y'], ob_data['Current_Acc_Z']])
            prev_acc = np.array([ob_data['Prev_Acc_X'], ob_data['Prev_Acc_Y'], ob_data['Prev_Acc_Z']])
            oldest_acc = np.array([ob_data['Oldest_Acc_X'], ob_data['Oldest_Acc_Y'], ob_data['Oldest_Acc_Z']])
            
            accelerations = [np.linalg.norm(current_acc), np.linalg.norm(prev_acc), np.linalg.norm(oldest_acc)]
            avg_acceleration = np.mean(accelerations)
            max_acceleration = np.max(accelerations)
            
            # 主导意图
            intent_cols = ['Intent_Forward', 'Intent_Left', 'Intent_Right', 'Intent_Stop']
            intent_values = [ob_data[col] for col in intent_cols]
            dominant_intent_idx = np.argmax(intent_values)
            dominant_intent = intent_cols[dominant_intent_idx].replace('Intent_', '')
            
            trajectory_stats.append({
                'Obstacle_ID': ob_id,
                'Trajectory_Length': trajectory_length,
                'Avg_Speed': avg_speed,
                'Max_Speed': max_speed,
                'Avg_Acceleration': avg_acceleration,
                'Max_Acceleration': max_acceleration,
                'Dominant_Intent': dominant_intent,
                'Num_Frames': ob_data['NumHistFrames']
            })
        
        traj_df = pd.DataFrame(trajectory_stats)
        
        if len(traj_df) > 0:
            print(f"轨迹统计 (基于 {len(traj_df)} 个障碍物):")
            print(f"  平均轨迹长度: {traj_df['Trajectory_Length'].mean():.2f}")
            print(f"  平均速度: {traj_df['Avg_Speed'].mean():.2f}")
            print(f"  最大速度: {traj_df['Max_Speed'].max():.2f}")
            print(f"  平均加速度: {traj_df['Avg_Acceleration'].mean():.2f}")
            print(f"  最大加速度: {traj_df['Max_Acceleration'].max():.2f}")
            print(f"  平均历史帧数: {traj_df['Num_Frames'].mean():.1f}")
            
            # 绘制轨迹分析图
            plt.figure(figsize=(12, 8))
            
            # 子图1: 速度分布
            plt.subplot(2, 2, 1)
            plt.hist(traj_df['Avg_Speed'], bins=20, alpha=0.7)
            plt.title('平均速度分布')
            plt.xlabel('速度')
            plt.ylabel('频次')
            
            # 子图2: 轨迹长度分布
            plt.subplot(2, 2, 2)
            plt.hist(traj_df['Trajectory_Length'], bins=20, alpha=0.7)
            plt.title('轨迹长度分布')
            plt.xlabel('长度')
            plt.ylabel('频次')
            
            # 子图3: 速度vs意图
            plt.subplot(2, 2, 3)
            for intent in traj_df['Dominant_Intent'].unique():
                intent_data = traj_df[traj_df['Dominant_Intent'] == intent]
                plt.scatter(intent_data['Avg_Speed'], intent_data['Trajectory_Length'], 
                           label=intent, alpha=0.7)
            plt.xlabel('平均速度')
            plt.ylabel('轨迹长度')
            plt.title('速度vs轨迹长度 (按意图分类)')
            plt.legend()
            
            # 子图4: 历史帧数分布
            plt.subplot(2, 2, 4)
            plt.hist(traj_df['Num_Frames'], bins=20, alpha=0.7)
            plt.title('历史帧数分布')
            plt.xlabel('帧数')
            plt.ylabel('频次')
            
            plt.tight_layout()
            plt.show()
    
    def analyze_temporal_patterns(self):
        """分析时间模式"""
        if self.data is None:
            return
        
        print("\n=== 时间模式分析 ===")
        
        # 按时间戳分组分析
        time_groups = self.data.groupby('Timestamp')
        
        timestamps = []
        obstacle_counts = []
        avg_intents = {
            'Forward': [],
            'Left': [],
            'Right': [],
            'Stop': []
        }
        
        for timestamp, group in time_groups:
            timestamps.append(timestamp)
            obstacle_counts.append(group['Obstacle_ID'].nunique())
            
            # 计算该时刻的平均意图概率
            avg_intents['Forward'].append(group['Intent_Forward'].mean())
            avg_intents['Left'].append(group['Intent_Left'].mean())
            avg_intents['Right'].append(group['Intent_Right'].mean())
            avg_intents['Stop'].append(group['Intent_Stop'].mean())
        
        # 绘制时间序列图
        plt.figure(figsize=(12, 8))
        
        # 子图1: 障碍物数量随时间变化
        plt.subplot(2, 1, 1)
        plt.plot(timestamps, obstacle_counts, 'b-o', markersize=4)
        plt.title('检测到的障碍物数量随时间变化')
        plt.ylabel('障碍物数量')
        plt.grid(True, alpha=0.3)
        
        # 子图2: 意图概率随时间变化
        plt.subplot(2, 1, 2)
        plt.plot(timestamps, avg_intents['Forward'], label='前进', linewidth=2)
        plt.plot(timestamps, avg_intents['Left'], label='左转', linewidth=2)
        plt.plot(timestamps, avg_intents['Right'], label='右转', linewidth=2)
        plt.plot(timestamps, avg_intents['Stop'], label='停止', linewidth=2)
        plt.title('平均意图概率随时间变化')
        plt.ylabel('概率')
        plt.xlabel('时间戳')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        print(f"时间跨度: {len(timestamps)} 个时间点")
        print(f"平均障碍物数量: {np.mean(obstacle_counts):.1f}")
    
    def generate_summary_report(self):
        """生成汇总报告"""
        if self.data is None:
            return
        
        print("\n" + "="*50)
        print("           意图数据分析汇总报告")
        print("="*50)
        
        # 基本统计信息
        print(f"数据文件: {self.csv_file_path}")
        print(f"总数据行数: {len(self.data)}")
        print(f"障碍物数量: {self.data['Obstacle_ID'].nunique()}")
        print(f"时间跨度: {self.data['Timestamp'].nunique()} 个时间点")
        
        # 意图统计
        latest_data = self.data.groupby('Obstacle_ID').last()
        intent_columns = ['Intent_Forward', 'Intent_Left', 'Intent_Right', 'Intent_Stop']
        dominant_intents = latest_data[intent_columns].idxmax(axis=1)
        
        print(f"\n主导意图分布:")
        for intent, count in dominant_intents.value_counts().items():
            intent_name = intent.replace('Intent_', '')
            percentage = count / len(dominant_intents) * 100
            print(f"  {intent_name}: {count} 个障碍物 ({percentage:.1f}%)")
        
        # 速度统计
        current_velocities = np.linalg.norm(self.data[['Current_Vel_X', 'Current_Vel_Y', 'Current_Vel_Z']].values, axis=1)
        print(f"\n速度统计:")
        print(f"  平均当前速度: {np.mean(current_velocities):.3f}")
        print(f"  最大当前速度: {np.max(current_velocities):.3f}")
        print(f"  最小当前速度: {np.min(current_velocities):.3f}")
        
        # 加速度统计
        current_accelerations = np.linalg.norm(self.data[['Current_Acc_X', 'Current_Acc_Y', 'Current_Acc_Z']].values, axis=1)
        print(f"\n加速度统计:")
        print(f"  平均当前加速度: {np.mean(current_accelerations):.3f}")
        print(f"  最大当前加速度: {np.max(current_accelerations):.3f}")
        print(f"  最小当前加速度: {np.min(current_accelerations):.3f}")
        
        # 历史帧数统计
        frame_counts = self.data.groupby('Obstacle_ID')['NumHistFrames'].first()
        print(f"\n历史帧数统计:")
        print(f"  平均帧数: {np.mean(frame_counts):.1f}")
        print(f"  最大帧数: {np.max(frame_counts)}")
        print(f"  最小帧数: {np.min(frame_counts)}")
        
        print("="*50)

def main():
    """主函数"""
    # 使用示例
    csv_file_path = "/tmp/intent_data.csv"
    
    print("意图数据分析工具")
    print(f"正在分析文件: {csv_file_path}")
    
    try:
        analyzer = IntentDataAnalyzer(csv_file_path)
        
        # 生成各种分析
        analyzer.generate_summary_report()
        analyzer.analyze_intent_distribution()
        analyzer.analyze_trajectory_patterns()
        analyzer.analyze_temporal_patterns()
        
    except FileNotFoundError:
        print(f"错误: 找不到文件 {csv_file_path}")
        print("请确保已启用数据导出功能并生成了数据文件")
    except Exception as e:
        print(f"分析过程中出现错误: {e}")

if __name__ == "__main__":
    main() 