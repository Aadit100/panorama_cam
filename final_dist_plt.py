import matplotlib.pyplot as plt

# Function to read files and extract final values
def read_final(file_paths):
    final_values = []
    for file_path in file_paths:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            final_value = 0
            for line in lines:
                if(line.strip()=='---'):
                    continue
                else:
                    value = float(line.strip())
                    final_value = value
            final_values.append(final_value)
    
    return final_values

file_paths = ['output_1.txt', 'output_2.txt', 'output_3.txt','output_4.txt','output_5.txt','output_6.txt','output_7.txt','output_8.txt','output_9.txt']  # Replace these with your file paths

final_values = read_final(file_paths)

x_values = range(1, len(final_values) + 1)

plt.figure(figsize=(8, 6))
plt.plot(x_values, final_values, marker='o', linestyle='-', color='red')
plt.xlabel('File Index')
plt.ylabel('Final Non-zero Value')
plt.title('Final Non-zero Values from Files')
plt.grid(True)
plt.show()
