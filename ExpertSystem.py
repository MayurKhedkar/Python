import datetime

class Patient:
  def __init__(self, name, age, gender, ph_no, email):
    self.name = name
    self.age = age
    self.gender = gender
    self.ph_no = ph_no
    self.email = email
    
class Doctor:
  def __init__(self, name, department, ph_no, email):
    self.name = name
    self.department = department
    self.ph_no = ph_no
    self.email = email
    
class Appointment:
  def __init__(self, doctor, patient, date, time):
    self.doctor = doctor
    self.patient = patient
    self.date = date
    self.time = time
    
class MedicalRecord:
  def __init__(self, doctor, patient, diagnosis, prescription):
    self.doctor = doctor
    self.patient = patient
    self.diagnosis = diagnosis
    self.prescription = prescription

class Hospital:
  def __init__(self, name, address, ph_no, email):
    self.name = name
    self.address = address
    self.ph_no = ph_no
    self.email = email
    self.patients = []
    self.doctors = []
    self.appointments = []
    self.medical_records = []
    
  def add_patient(self, patient):
    self.patients.append(patient)
    
  def add_doctor(self, doctor):
    self.doctors.append(doctor)
  
  def schedule_appointment(self, doctor, patient, date, time):
    appointment = Appointment(doctor, patient, date, time)
    self.appointments.append(appointment)
  
  def add_medical_record(self, record):
    self.medical_records.append(record)

  def find_doctor_by_department(self, department):
    matching_doctors = []
    for doctor in self.doctors:
      if doctor.department == department:
        matching_doctors.append(doctor)
    return matching_doctors
  
  def find_appointment_by_doctor_and_date(self, doctor, date):
    matching_appointments = []
    for appointment in self.appointments:
      if appointment.doctor == doctor and appointment.date == date:
        matching_appointments.append(appointment)
    return matching_appointments
  
  def find_medical_records_by_patient(self, patient):
    matching_medical_records = []
    for medical_record in self.medical_records:
      if medical_record.patient == patient:
        matching_medical_records.append(medical_record)
    return matching_medical_records
  
hospital = Hospital("KEM Hospital", "Rasta Peth", "555-666", "KEMHospital@gmail.com")

doctor1 = Doctor("Dr. Deshmukh", "Cardiologist", "444-777", "Deshmukh47@gmail.com")
doctor2 = Doctor("Dr. Godbole", "Pediatrics", "222-999", "Godbole29@gmail.com")
hospital.add_doctor(doctor1)
hospital.add_doctor(doctor2)

patient1 = Patient("Pratik", 25, "Male", "333-888", "Pratik38@gmail.com")
patient2 = Patient("Deepa", 28, "Female", "666-444", "Deepa64@gmail.com")
hospital.add_patient(patient1)
hospital.add_patient(patient2)

date1 = datetime.date(2023, 5, 5)
time1 = datetime.time(4, 30)
hospital.schedule_appointment(doctor1, patient1, date1, time1)

date2 = datetime.date(2023, 5, 6)
time2 = datetime.time(10, 0)
hospital.schedule_appointment(doctor2, patient2, date2, time2)

medical_record1 = MedicalRecord(doctor1, patient1, "Diagnosis1", "Prescription1")
hospital.add_medical_record(medical_record1)

medical_record2 = MedicalRecord(doctor2, patient2, "Diagnosis2", "Prescription2")
hospital.add_medical_record(medical_record2)

cardiologists = hospital.find_doctor_by_department("Cardilogist")
print("Cardiologist:")
for doctor in cardiologists:
  print(doctor.name)

appointments = hospital.find_appointment_by_doctor_and_date(doctor1, date1)
print("Appointments for Dr. Deshmukh on 5th May 2023 are")
for appointment in appointments:
  print(appointment.patient.name)

medical_records = hospital.find_medical_records_by_patient(patient2)
print("Medical Records for Deepa are")
for medical_record in medical_records:
  print("Diagnosis", medical_record.diagnosis)
  print("Prescription", medical_record.prescription)


