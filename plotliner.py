import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os

# --- 1. Configuration ---
files_to_compare = {
    'avg-1L3H.csv': '1L3H',
    'avg-3L1H.csv': '3L1H',
    'avg-Anchor12-H0.csv': '2L2H',
    'avg-Anchor2-H12.csv': 'LMHH',
    'avg-LH2M.csv': 'LH2M',
    # 'avg-Static-H3.csv': 'Static-H3',
    # 'avg-Static-H6.csv': 'Static-H6'
}

# --- 2. Data Loading ---
dataframes = {}
loaded_files = []
for file_path, label in files_to_compare.items():
    if os.path.exists(file_path):
        print(f"กำลังอ่านไฟล์: {file_path}")
        dataframes[label] = pd.read_csv(file_path)
        loaded_files.append(label)
    else:
        print(f"ไม่พบไฟล์: {file_path} - จะข้ามไฟล์นี้")

if not dataframes:
    print("ไม่สามารถโหลดไฟล์ CSV ใดๆ ได้เลย")
else:
    print(f"โหลดข้อมูลเรียบร้อยแล้ว {len(dataframes)} ไฟล์: {', '.join(loaded_files)}")
    
    # --- 3. Calculate Comprehensive Statistics ---
    summary_data = []
    for label, df in dataframes.items():
        required_cols = ['err_3d_cm', 'err_xy_cm', 'err_x_cm', 'err_y_cm', 'err_z_cm']
        if all(col in df.columns for col in required_cols):
            
            rmse_x = np.sqrt(np.mean(df['err_x_cm']**2))
            rmse_y = np.sqrt(np.mean(df['err_y_cm']**2))
            rmse_z = np.sqrt(np.mean(df['err_z_cm']**2))
            
            stats = {
                'Configuration': label,
                'Mean 3D': df['err_3d_cm'].mean(),
                'Mean XY': df['err_xy_cm'].mean(),
                'Mean X Abs': df['err_x_cm'].abs().mean(),
                'Mean Y Abs': df['err_y_cm'].abs().mean(),
                'Mean Z Abs': df['err_z_cm'].abs().mean(), # ใช้ค่านี้ในการจัดเรียง
                'RMSE Z': rmse_z
            }
            summary_data.append(stats)
        else:
            print(f"ไฟล์ {label} ขาดคอลัมน์ที่จำเป็น")

    # --- 4. Create and Save Summary Table as Image (แก้ไขใหม่) ---
    summary_df = pd.DataFrame(summary_data)
    if not summary_df.empty:
        # จัดเรียงตาม Mean Z Abs Error (น้อยไปมาก)
        summary_df = summary_df.sort_values(by='Mean Z Abs', ascending=True)
        
        # เตรียมข้อมูลสำหรับ Plot Table
        # ปัดเศษทศนิยมให้เหลือ 2 ตำแหน่งเพื่อให้ดูง่าย
        plot_df = summary_df.round(2) 
        
        # สร้างรูปภาพสำหรับตาราง
        plt.figure(figsize=(12, 4)) # ปรับขนาดภาพตามความเหมาะสม
        ax = plt.gca()
        ax.axis('off') # ซ่อนแกน x, y

        # สร้างตาราง
        # cellText: ข้อมูลในตาราง (ไม่เอา index)
        # colLabels: ชื่อหัวข้อคอลัมน์
        # rowLabels: ชื่อ Config (ใช้ค่าจากคอลัมน์ Configuration)
        table_data = plot_df.drop(columns=['Configuration'])
        row_labels = plot_df['Configuration']
        
        the_table = plt.table(cellText=table_data.values,
                              colLabels=table_data.columns,
                              rowLabels=row_labels.values,
                              cellLoc='center',
                              loc='center')

        # ปรับแต่งความสวยงามของตาราง
        the_table.auto_set_font_size(False)
        the_table.set_fontsize(11)
        the_table.scale(1.2, 1.5) # ปรับขนาดเซลล์ (กว้าง, สูง)

        # ไฮไลท์: แถวบนสุด (Best Case) ให้เป็นสีเขียวอ่อน
        for k, cell in the_table.get_celld().items():
            row, col = k
            # แถวที่ 0 คือ Header, แถวที่ 1 คือข้อมูลแถวแรก (ดีที่สุด)
            if row == 0:
                cell.set_text_props(weight='bold', color='white')
                cell.set_facecolor('#40466e') # สีหัวตาราง (น้ำเงินเข้ม)
            elif row == 1: 
                cell.set_facecolor('#dbf2d9') # สีเขียวอ่อนสำหรับค่าที่ดีที่สุด
            else:
                cell.set_facecolor('#f5f5f5') # สีพื้นหลังทั่วไป

        plt.title('Performance Summary (Sorted by Z-Axis Error)', fontweight="bold", pad=20)
        plt.tight_layout()
        plt.savefig('performance_summary_table.png', dpi=300, bbox_inches='tight')
        plt.clf()
        print("\n--- สร้างไฟล์รูปภาพตาราง 'performance_summary_table.png' เรียบร้อยแล้ว ---")
        
        # แสดงผลแบบ Text ใน Console ด้วย (เผื่อไว้)
        print(summary_df.set_index('Configuration').to_string())

        # --- 5. Create Bar Charts (เหมือนเดิมแต่ปรับชื่อคอลัมน์ให้ตรง) ---
        colors_list = ['skyblue', 'salmon', 'lightgreen', 'coral', 'orchid']

        # ตัวอย่าง Plot 1: Mean 3D Error
        plt.figure(figsize=(10, 6))
        # ต้อง set index เป็น Configuration เพื่อให้ชื่อขึ้นที่แกน X
        summary_df.set_index('Configuration')['Mean 3D'].plot(kind='bar', color=colors_list)
        plt.title('Comparison: Mean 3D Error (Lower is Better)', fontsize=14)
        plt.ylabel('Error (cm)', fontsize=12)
        plt.xticks(rotation=45)
        plt.grid(axis='y', linestyle=':', alpha=0.7)
        plt.tight_layout()
        plt.savefig('summary_bar_all_3d_error.png')
        plt.clf()
        print("สร้างกราฟแท่ง 3D Error เรียบร้อย")

        # (คุณสามารถเพิ่มกราฟอื่นๆ ตรงนี้ตามโค้ดเดิม โดยใช้ชื่อคอลัมน์ใหม่ที่สั้นลง)
        
        # --- 6. Plot XY Path (Best Config) ---
        best_config_label = summary_df.iloc[0]['Configuration'] # เอาตัวแรกสุดหลัง sort
        best_df = dataframes[best_config_label]
        
        print(f"\nกำลังสร้างกราฟเส้นทาง XY ของ Config ที่ดีที่สุด: {best_config_label}")
        
        plt.figure(figsize=(8, 8))
        plt.plot(best_df['gt_x_cm'], best_df['gt_y_cm'], 'b--o', label='Ground Truth', markersize=4)
        plt.plot(best_df['avg_x_cm'], best_df['avg_y_cm'], 'r-x', label=f'Measured ({best_config_label})', markersize=4)
        
        # วาด Label P0, P1...
        for i, row in best_df.iterrows():
             if i % 5 == 0: # ลดความรก ใส่ label ทุกๆ 5 จุด (หรือเอา if ออกถ้าจุดน้อย)
                plt.text(row['gt_x_cm'], row['gt_y_cm'], f'{i}', color='blue', fontsize=8)
        
        plt.title(f'Path Comparison: {best_config_label}')
        plt.xlabel('X (cm)')
        plt.ylabel('Y (cm)')
        plt.legend()
        plt.grid(True, linestyle=':')
        plt.axis('equal')
        plt.tight_layout()
        plt.savefig('best_config_path.png')
        plt.clf()
        print("สร้างกราฟ Path เรียบร้อย")

    else:
        print("ไม่มีข้อมูลสรุป")