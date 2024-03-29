import numpy as np
import pandas as pd
from sklearn.feature_selection import SelectKBest
from sklearn.feature_selection import f_classif
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, classification_report
from micromlgen import port
import joblib

data = pd.read_csv(
    '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/all.txt', header=None)

# PREPROCESSING

# F - forward
# I - forward right
# R - right
# G - forward left
# L - left

# get rid of rows labeled L, H, R, J
data.rename(columns={data.columns[-1]: 'Label'}, inplace=True)
data = data[(data['Label'] != 'L') & (data['Label'] != 'R') &
            (data['Label'] != 'H') & (data['Label'] != 'J')]
data.reset_index(drop=True, inplace=True)
X = data.iloc[:, :-1]
y = data.iloc[:, -1]

label_encoder = LabelEncoder()
y = label_encoder.fit_transform(y)

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42)

k = 80
k_best = SelectKBest(score_func=f_classif, k=k)
k_best.fit(X_train, y_train)

selected_feature_indices = k_best.get_support(indices=True)
print("selected_feature_indices: ", selected_feature_indices)

clf = RandomForestClassifier(max_depth=3, random_state=42)
clf.fit(X_train.iloc[:, selected_feature_indices], y_train)

y_pred = clf.predict(X_test.iloc[:, selected_feature_indices])

accuracy = accuracy_score(y_test, y_pred)
print(f'Accuracy: {accuracy}')

class_names = label_encoder.classes_
report = classification_report(
    y_test, y_pred, target_names=class_names, zero_division=0)
print('Classification Report:\n', report)

# arduino_code = open("arduino_random_forest10.c", mode="w+")
# arduino_code.write(port(clf))

# Save the trained model
joblib.dump(
    clf, '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/rf_model/random_forest_model.pkl')

# Save the feature selector
joblib.dump(
    k_best, '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/k_best/k_best.pkl')

# Save the label encoder
joblib.dump(label_encoder,
            '/media/psf/Developer/Robotics/char-01/ros2_pkg/data/label_encoder/label_encoder.pkl')
