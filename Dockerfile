# Use an official Python runtime as a base image
FROM python:3.13-slim

# Set the working directory in the container
WORKDIR /app

# Copy the requirements file to the working directory
COPY requirements.txt .

# Install any dependencies listed in requirements.txt
RUN pip install --no-cache-dir -r requirements.txt

# Copy the entire project directory into the container
COPY . .

# Set the default command to run your Python program
CMD ["python", "src/main.py"]
