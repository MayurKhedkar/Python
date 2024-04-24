class InformationManagementExpertSystem:
    def __init__(self):
        self.data = {}

    def add_information(self, key, value):
        self.data[key] = value
        print(f"Information '{key}' added successfully.")

    def get_information(self, key):
        if key in self.data:
            return self.data[key]
        else:
            print(f"Information '{key}' not found.")

    def remove_information(self, key):
        if key in self.data:
            del self.data[key]
            print(f"Information '{key}' removed successfully.")
        else:
            print(f"Information '{key}' not found.")

    def display_all_information(self):
        if self.data:
            print("All information:")
            for key, value in self.data.items():
                print(f"{key}: {value}")
        else:
            print("No information available.")

# Usage example
expert_system = InformationManagementExpertSystem()
expert_system.add_information("Name", "John Doe")
expert_system.add_information("Age", 30)
expert_system.display_all_information()
print(expert_system.get_information("Name"))
expert_system.remove_information("Age")
