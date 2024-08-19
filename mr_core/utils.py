    def calculate_progress(self, waypoints, current_x, current_y):
        if not waypoints:
            return 0.0

        current_pose = UTMPose(current_x, current_y)
        total_distance = 0.0
        distance_covered = 0.0

        for i in range(1, len(waypoints)):
            total_distance += self.calculate_distance(waypoints[i-1], waypoints[i])

        closest_index = min(range(len(waypoints)), key=lambda i: self.calculate_distance(current_pose, waypoints[i]))

        for i in range(1, closest_index + 1):
            distance_covered += self.calculate_distance(waypoints[i-1], waypoints[i])

        if closest_index < len(waypoints) - 1:
            distance_covered += self.calculate_distance(waypoints[closest_index], current_pose)

        return distance_covered / total_distance if total_distance > 0 else 0.0

    @staticmethod
    def calculate_distance(pose1, pose2):
        return math.sqrt((pose1.x - pose2.x) ** 2 + (pose1.y - pose2.y) ** 2)
