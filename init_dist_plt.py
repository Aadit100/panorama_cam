import matplotlib.pyplot as plt

# Function to read files and extract first non-zero values
def read_first_non_zero(file_paths):
    first_values = []
    for file_path in file_paths:
        with open(file_path, 'r') as file:
            lines = file.readlines()
            found = False
            for line in lines:
                if(line.strip()=='---'):
                    value=0
                else:
                    value = float(line.strip())
                if value != 0:
                    first_values.append(value)
                    found = True
                    break
            if not found:
                first_values.append(0)
    
    return first_values

file_paths = ['output_1.txt', 'output_2.txt', 'output_3.txt','output_4.txt','output_5.txt','output_6.txt','output_7.txt','output_8.txt','output_9.txt']  # Replace these with your file paths

first_values = read_first_non_zero(file_paths)

x_values = range(1, len(first_values) + 1)

plt.figure(figsize=(8, 6))
plt.plot(x_values, first_values, marker='o', linestyle='-', color='blue')
plt.xlabel('File Index')
plt.ylabel('First Non-zero Value')
plt.title('First Non-zero Values from Files')
plt.grid(True)
plt.show()