export interface Exercise {
  id: string;
  name: string;
  videoUrl: string;
  form: "bad" | "medium" | "good";
  date: string;
  duration: number;
}

class ExerciseStore {
  private exercises: Exercise[] = [
    {
      id: "0da5d038-e673-44d3-b244-913af1fb460c",
      name: "Battle Ropes 89",
      videoUrl: "https://fitnessvids.com/exercise-battle-ropes-986.mp4",
      form: "bad",
      date: "2025-03-25T09:11:08.551Z",
      duration: 465,
    },
    {
      id: "70b749c9-5281-48e0-8c50-cc51081e8d44",
      name: "Russian Twists 14",
      videoUrl: "https://workoutlibrary.net/videos/russian-twists-415.mp4",
      form: "good",
      date: "2025-02-11T09:11:08.555Z",
      duration: 512,
    },
    {
      id: "a7cc9845-ff23-4dfe-8735-83c3ef76a286",
      name: "Jumping Jacks 63",
      videoUrl: "https://exercisedb.org/video/jumping-jacks-269.mp4",
      form: "good",
      date: "2025-03-13T09:11:08.555Z",
      duration: 477,
    },
    {
      id: "6df966d9-35c0-42e8-9d40-b642dc22d5b7",
      name: "Flutter Kicks 57",
      videoUrl: "https://workoutlibrary.net/videos/flutter-kicks-986.mp4",
      form: "medium",
      date: "2025-03-19T09:11:08.555Z",
      duration: 178,
    },
    {
      id: "85b11279-ecdd-4c85-99ca-f31bb50ec0ba",
      name: "Flutter Kicks 2",
      videoUrl: "https://fitnessvids.com/exercise-flutter-kicks-582.mp4",
      form: "medium",
      date: "2025-02-22T09:11:08.555Z",
      duration: 589,
    },
    {
      id: "6ad4d16a-612b-406a-bde2-7352ccc7c0ae",
      name: "Flutter Kicks 34",
      videoUrl: "https://exercisedb.org/video/flutter-kicks-303.mp4",
      form: "medium",
      date: "2025-02-23T09:11:08.555Z",
      duration: 562,
    },
    {
      id: "42b44f64-83df-46e3-9318-5ff4eeb21172",
      name: "Pull-ups 89",
      videoUrl: "https://trainingmedia.com/exercises/pull-ups-160.mp4",
      form: "good",
      date: "2025-03-20T09:11:08.555Z",
      duration: 554,
    },
    {
      id: "66bb1b24-ed6c-474b-b43a-001e71c751f9",
      name: "Hip Thrusts 86",
      videoUrl: "https://fitnessvids.com/exercise-hip-thrusts-562.mp4",
      form: "medium",
      date: "2025-02-12T09:11:08.555Z",
      duration: 202,
    },
    {
      id: "857338e8-d770-4d05-b63d-283df9240ca7",
      name: "Hip Thrusts 76",
      videoUrl: "https://workoutlibrary.net/videos/hip-thrusts-809.mp4",
      form: "bad",
      date: "2025-03-24T09:11:08.555Z",
      duration: 167,
    },
    {
      id: "9558b504-22e5-43f4-bf1e-c9b119dadc8e",
      name: "Crunches 60",
      videoUrl: "https://trainingmedia.com/exercises/crunches-66.mp4",
      form: "bad",
      date: "2025-04-04T08:11:08.555Z",
      duration: 465,
    },
    {
      id: "90872790-4ff0-4680-abfd-b1e021b796f7",
      name: "TRX Rows 85",
      videoUrl: "https://exercisedb.org/video/trx-rows-577.mp4",
      form: "good",
      date: "2025-03-22T09:11:08.555Z",
      duration: 588,
    },
    {
      id: "948ddadb-d973-4c1f-b451-8e2d02fb72b7",
      name: "Lateral Raises 39",
      videoUrl: "https://fitnessvids.com/exercise-lateral-raises-398.mp4",
      form: "good",
      date: "2025-04-02T08:11:08.555Z",
      duration: 207,
    },
    {
      id: "1b7d21be-8ebe-4d45-a0b5-dd467c4ea330",
      name: "Jumping Jacks 42",
      videoUrl: "https://fitnessvids.com/exercise-jumping-jacks-910.mp4",
      form: "good",
      date: "2025-02-06T09:11:08.555Z",
      duration: 543,
    },
    {
      id: "c8f8175f-4fae-44e9-8759-29be1268a6d0",
      name: "Leg Raises 72",
      videoUrl: "https://exercisedb.org/video/leg-raises-330.mp4",
      form: "medium",
      date: "2025-02-26T09:11:08.555Z",
      duration: 347,
    },
    {
      id: "e51997dc-d72c-4bac-acd1-a7c3f9c741a3",
      name: "Medicine Ball Throws 60",
      videoUrl: "https://fitnessvids.com/exercise-medicine-ball-throws-139.mp4",
      form: "good",
      date: "2025-03-19T09:11:08.555Z",
      duration: 137,
    },
    {
      id: "1c0a0335-eb30-4234-a8e5-753595334e18",
      name: "Plank 42",
      videoUrl: "https://exercisedb.org/video/plank-537.mp4",
      form: "medium",
      date: "2025-02-17T09:11:08.555Z",
      duration: 495,
    },
    {
      id: "53d9d501-71b4-4452-8971-347852cbb639",
      name: "Plank 69",
      videoUrl: "https://trainingmedia.com/exercises/plank-964.mp4",
      form: "bad",
      date: "2025-03-22T09:11:08.555Z",
      duration: 486,
    },
    {
      id: "80a33261-0b1d-46a6-827f-4f28c220e266",
      name: "Dips 85",
      videoUrl: "https://workoutlibrary.net/videos/dips-628.mp4",
      form: "bad",
      date: "2025-02-27T09:11:08.555Z",
      duration: 330,
    },
    {
      id: "ee5a0d00-db67-4017-9e20-c6ffcd9217c0",
      name: "Deadlifts 39",
      videoUrl: "https://trainingmedia.com/exercises/deadlifts-975.mp4",
      form: "bad",
      date: "2025-03-06T09:11:08.555Z",
      duration: 102,
    },
    {
      id: "4fde66c3-7d81-4de3-8010-6d61b3829c4e",
      name: "Box Jumps 13",
      videoUrl: "https://workoutlibrary.net/videos/box-jumps-841.mp4",
      form: "bad",
      date: "2025-03-23T09:11:08.555Z",
      duration: 246,
    },
    {
      id: "da532051-d921-4087-b20c-c456e455a2f5",
      name: "Bench Press 78",
      videoUrl: "https://trainingmedia.com/exercises/bench-press-413.mp4",
      form: "good",
      date: "2025-02-10T09:11:08.555Z",
      duration: 523,
    },
    {
      id: "2a0bab74-a3a7-402c-97eb-aafcbc1888b1",
      name: "Preacher Curls 21",
      videoUrl: "https://workoutlibrary.net/videos/preacher-curls-589.mp4",
      form: "good",
      date: "2025-02-28T09:11:08.555Z",
      duration: 192,
    },
    {
      id: "1ecc7a72-5be4-402c-90a6-d8da55f45178",
      name: "Side Planks 88",
      videoUrl: "https://fitnessvids.com/exercise-side-planks-273.mp4",
      form: "good",
      date: "2025-02-18T09:11:08.555Z",
      duration: 367,
    },
    {
      id: "178a8768-1822-4d07-86e9-31cd38aacca5",
      name: "Lunges 59",
      videoUrl: "https://workoutlibrary.net/videos/lunges-504.mp4",
      form: "good",
      date: "2025-02-17T09:11:08.555Z",
      duration: 64,
    },
    {
      id: "4d1f35e1-c89d-4b1d-af42-601e131ee281",
      name: "Bicep Curls 80",
      videoUrl: "https://exercisedb.org/video/bicep-curls-464.mp4",
      form: "good",
      date: "2025-02-27T09:11:08.555Z",
      duration: 88,
    },
    {
      id: "43be0a24-fc1a-438c-bcc1-dd7a4f54e279",
      name: "Leg Raises 49",
      videoUrl: "https://fitnessvids.com/exercise-leg-raises-668.mp4",
      form: "bad",
      date: "2025-03-07T09:11:08.555Z",
      duration: 239,
    },
    {
      id: "ecddd68c-15c3-4ef0-824e-0a36e61ffa35",
      name: "Bench Press 84",
      videoUrl: "https://fitnessvids.com/exercise-bench-press-645.mp4",
      form: "good",
      date: "2025-03-08T09:11:08.555Z",
      duration: 145,
    },
    {
      id: "9c67a79f-661c-4743-a876-f8abb5ded19d",
      name: "TRX Rows 53",
      videoUrl: "https://trainingmedia.com/exercises/trx-rows-427.mp4",
      form: "good",
      date: "2025-02-28T09:11:08.555Z",
      duration: 304,
    },
    {
      id: "355d4c5c-599c-42d4-8c39-eb614d3a7544",
      name: "High Knees 77",
      videoUrl: "https://workoutlibrary.net/videos/high-knees-452.mp4",
      form: "bad",
      date: "2025-02-06T09:11:08.555Z",
      duration: 141,
    },
    {
      id: "31c6436d-fff6-4199-809b-60d96d1bf945",
      name: "Deadlifts 10",
      videoUrl: "https://exercisedb.org/video/deadlifts-547.mp4",
      form: "good",
      date: "2025-03-04T09:11:08.555Z",
      duration: 541,
    },
    {
      id: "3b2bd5f5-8f7b-4491-84f7-5a3ef85f87ca",
      name: "Preacher Curls 93",
      videoUrl: "https://fitnessvids.com/exercise-preacher-curls-872.mp4",
      form: "bad",
      date: "2025-02-19T09:11:08.555Z",
      duration: 72,
    },
    {
      id: "bc438edf-60c0-4553-8aeb-9de10d3a8a22",
      name: "Medicine Ball Throws 62",
      videoUrl:
        "https://trainingmedia.com/exercises/medicine-ball-throws-775.mp4",
      form: "bad",
      date: "2025-03-26T09:11:08.555Z",
      duration: 94,
    },
    {
      id: "b2bd17b3-e385-4094-aaef-c500be1e3f58",
      name: "Front Raises 36",
      videoUrl: "https://trainingmedia.com/exercises/front-raises-674.mp4",
      form: "bad",
      date: "2025-03-14T09:11:08.555Z",
      duration: 514,
    },
    {
      id: "cb8062a3-55d2-4513-a54f-c64c2a75ab17",
      name: "Squats 37",
      videoUrl: "https://fitnessvids.com/exercise-squats-893.mp4",
      form: "bad",
      date: "2025-03-25T09:11:08.555Z",
      duration: 546,
    },
    {
      id: "a8988745-f0a4-4ded-bb5e-0fc3be7cfc51",
      name: "Cable Pushdowns 13",
      videoUrl: "https://fitnessvids.com/exercise-cable-pushdowns-612.mp4",
      form: "good",
      date: "2025-03-15T09:11:08.555Z",
      duration: 126,
    },
    {
      id: "d2c9654d-4169-411c-b9a9-5f31c3a997d3",
      name: "Front Raises 83",
      videoUrl: "https://trainingmedia.com/exercises/front-raises-332.mp4",
      form: "medium",
      date: "2025-03-29T09:11:08.555Z",
      duration: 538,
    },
    {
      id: "a917b4c1-76a7-42a0-abbc-401106bc7d9d",
      name: "Shrugs 32",
      videoUrl: "https://trainingmedia.com/exercises/shrugs-972.mp4",
      form: "bad",
      date: "2025-03-24T09:11:08.555Z",
      duration: 469,
    },
    {
      id: "294774ea-e19d-4c89-be94-bc4a471504fd",
      name: "Glute Bridges 73",
      videoUrl: "https://fitnessvids.com/exercise-glute-bridges-430.mp4",
      form: "bad",
      date: "2025-02-24T09:11:08.555Z",
      duration: 306,
    },
    {
      id: "aaa94668-6925-4ee9-8a42-07e2cfb6ad3f",
      name: "Push-ups 2",
      videoUrl: "https://fitnessvids.com/exercise-push-ups-979.mp4",
      form: "medium",
      date: "2025-02-16T09:11:08.555Z",
      duration: 95,
    },
    {
      id: "c7c76f1e-8c43-4740-bb71-f4a1a6ea0154",
      name: "Burpees 35",
      videoUrl: "https://exercisedb.org/video/burpees-405.mp4",
      form: "medium",
      date: "2025-03-31T08:11:08.555Z",
      duration: 464,
    },
    {
      id: "fb75bcd3-6d08-4244-83bf-a8f25f5cc778",
      name: "Tricep Extensions 80",
      videoUrl: "https://trainingmedia.com/exercises/tricep-extensions-191.mp4",
      form: "medium",
      date: "2025-03-24T09:11:08.555Z",
      duration: 296,
    },
    {
      id: "110f474e-8765-43b7-8e9b-444758d66041",
      name: "Calf Raises 11",
      videoUrl: "https://workoutlibrary.net/videos/calf-raises-497.mp4",
      form: "medium",
      date: "2025-03-17T09:11:08.555Z",
      duration: 551,
    },
    {
      id: "6905bc31-64d4-421d-940a-f5e4d2cfb271",
      name: "Jumping Rope 58",
      videoUrl: "https://fitnessvids.com/exercise-jumping-rope-429.mp4",
      form: "medium",
      date: "2025-04-01T08:11:08.555Z",
      duration: 513,
    },
    {
      id: "524b52a8-ae9e-448f-9623-8b2e2151d46d",
      name: "Burpees 78",
      videoUrl: "https://exercisedb.org/video/burpees-903.mp4",
      form: "good",
      date: "2025-03-15T09:11:08.555Z",
      duration: 393,
    },
    {
      id: "38c25078-00f0-4785-bab9-27771b82a862",
      name: "Box Jumps 74",
      videoUrl: "https://exercisedb.org/video/box-jumps-35.mp4",
      form: "good",
      date: "2025-03-01T09:11:08.555Z",
      duration: 578,
    },
    {
      id: "bd9b7a2c-8fbf-4e6a-97bf-e2c6bd833624",
      name: "Squats 91",
      videoUrl: "https://trainingmedia.com/exercises/squats-127.mp4",
      form: "medium",
      date: "2025-03-21T09:11:08.555Z",
      duration: 471,
    },
    {
      id: "edc45f4d-e212-4e95-b600-0bfcb77fb229",
      name: "Lat Pulldowns 86",
      videoUrl: "https://fitnessvids.com/exercise-lat-pulldowns-940.mp4",
      form: "good",
      date: "2025-02-13T09:11:08.555Z",
      duration: 573,
    },
    {
      id: "b91b837b-9203-4461-aba1-4e426d518e5d",
      name: "Face Pulls 89",
      videoUrl: "https://workoutlibrary.net/videos/face-pulls-291.mp4",
      form: "bad",
      date: "2025-03-21T09:11:08.555Z",
      duration: 489,
    },
    {
      id: "febb28ed-c996-4d58-8127-2bc51d371be8",
      name: "Front Raises 76",
      videoUrl: "https://exercisedb.org/video/front-raises-532.mp4",
      form: "medium",
      date: "2025-03-21T09:11:08.555Z",
      duration: 42,
    },
    {
      id: "dd7cde43-978e-4dc9-9d3f-58652a51cbe3",
      name: "Calf Raises 11",
      videoUrl: "https://workoutlibrary.net/videos/calf-raises-948.mp4",
      form: "bad",
      date: "2025-02-20T09:11:08.555Z",
      duration: 70,
    },
    {
      id: "ac7cab11-2e80-4b98-9a03-f1d222e4bc3b",
      name: "Box Jumps 31",
      videoUrl: "https://fitnessvids.com/exercise-box-jumps-839.mp4",
      form: "good",
      date: "2025-03-12T09:11:08.555Z",
      duration: 52,
    },
    {
      id: "0a4513e9-2973-47b1-9482-7c4e36d95c46",
      name: "Bench Press 68",
      videoUrl: "https://workoutlibrary.net/videos/bench-press-633.mp4",
      form: "good",
      date: "2025-03-12T09:11:08.555Z",
      duration: 575,
    },
    {
      id: "8537ff42-9490-49d2-ae51-b3820b7a9e3c",
      name: "Wall Sits 83",
      videoUrl: "https://workoutlibrary.net/videos/wall-sits-410.mp4",
      form: "good",
      date: "2025-04-01T08:11:08.555Z",
      duration: 273,
    },
    {
      id: "47363297-097f-44f0-bf99-d1db944abc3a",
      name: "Flutter Kicks 89",
      videoUrl: "https://trainingmedia.com/exercises/flutter-kicks-352.mp4",
      form: "bad",
      date: "2025-03-12T09:11:08.555Z",
      duration: 71,
    },
    {
      id: "1f23e13f-bfaf-4863-9890-931aad615986",
      name: "Bicep Curls 17",
      videoUrl: "https://workoutlibrary.net/videos/bicep-curls-25.mp4",
      form: "good",
      date: "2025-03-13T09:11:08.555Z",
      duration: 398,
    },
    {
      id: "aaf0b02c-454a-43bb-86f4-baddef6b4124",
      name: "Burpees 75",
      videoUrl: "https://trainingmedia.com/exercises/burpees-931.mp4",
      form: "bad",
      date: "2025-02-27T09:11:08.555Z",
      duration: 505,
    },
    {
      id: "fa0a7074-5d00-4bf1-947f-fd23bdf75df1",
      name: "Battle Ropes 52",
      videoUrl: "https://exercisedb.org/video/battle-ropes-476.mp4",
      form: "good",
      date: "2025-02-09T09:11:08.555Z",
      duration: 256,
    },
    {
      id: "e1f7a527-4685-4894-b94a-9dd6abaff085",
      name: "Leg Press 55",
      videoUrl: "https://trainingmedia.com/exercises/leg-press-942.mp4",
      form: "medium",
      date: "2025-02-24T09:11:08.555Z",
      duration: 543,
    },
    {
      id: "9f845c07-46a9-44a8-bad2-693e4563b44d",
      name: "Preacher Curls 94",
      videoUrl: "https://trainingmedia.com/exercises/preacher-curls-767.mp4",
      form: "bad",
      date: "2025-03-20T09:11:08.555Z",
      duration: 540,
    },
    {
      id: "83e94ba3-d8ea-4c34-9f9e-cfb25cced50c",
      name: "Leg Press 5",
      videoUrl: "https://exercisedb.org/video/leg-press-467.mp4",
      form: "medium",
      date: "2025-02-08T09:11:08.555Z",
      duration: 541,
    },
    {
      id: "539074dd-e6d8-4fc2-9353-975ef41cfce4",
      name: "Hip Thrusts 90",
      videoUrl: "https://fitnessvids.com/exercise-hip-thrusts-17.mp4",
      form: "bad",
      date: "2025-03-08T09:11:08.555Z",
      duration: 480,
    },
    {
      id: "07b3c528-c4f4-4569-9124-5f5b9fcdc66e",
      name: "Hammer Curls 40",
      videoUrl: "https://workoutlibrary.net/videos/hammer-curls-3.mp4",
      form: "bad",
      date: "2025-04-03T08:11:08.555Z",
      duration: 255,
    },
    {
      id: "241d5aff-759e-4bc7-940e-bd216d53df94",
      name: "Medicine Ball Throws 13",
      videoUrl: "https://fitnessvids.com/exercise-medicine-ball-throws-372.mp4",
      form: "medium",
      date: "2025-03-10T09:11:08.555Z",
      duration: 121,
    },
    {
      id: "d76b64b3-c16f-44f8-a442-13187ec10c74",
      name: "Leg Press 31",
      videoUrl: "https://exercisedb.org/video/leg-press-284.mp4",
      form: "medium",
      date: "2025-03-20T09:11:08.555Z",
      duration: 392,
    },
    {
      id: "d1c0ab79-b85a-4ebc-a207-d18e4b84b484",
      name: "Pull-ups 2",
      videoUrl: "https://trainingmedia.com/exercises/pull-ups-243.mp4",
      form: "bad",
      date: "2025-03-03T09:11:08.555Z",
      duration: 377,
    },
    {
      id: "89fa10d2-03c6-43ed-b8e4-76a822d82d28",
      name: "Shoulder Press 58",
      videoUrl: "https://workoutlibrary.net/videos/shoulder-press-259.mp4",
      form: "good",
      date: "2025-03-03T09:11:08.555Z",
      duration: 391,
    },
    {
      id: "ff1074af-262d-4c72-877c-ffc184498335",
      name: "Burpees 85",
      videoUrl: "https://exercisedb.org/video/burpees-696.mp4",
      form: "bad",
      date: "2025-02-18T09:11:08.555Z",
      duration: 269,
    },
    {
      id: "0923bd63-4f20-4e84-9b52-0271469467f2",
      name: "Battle Ropes 87",
      videoUrl: "https://trainingmedia.com/exercises/battle-ropes-386.mp4",
      form: "bad",
      date: "2025-04-02T08:11:08.555Z",
      duration: 266,
    },
    {
      id: "bca916d0-84f4-4fe8-9b4c-a3e9776a79ee",
      name: "Tricep Extensions 54",
      videoUrl: "https://fitnessvids.com/exercise-tricep-extensions-944.mp4",
      form: "medium",
      date: "2025-03-17T09:11:08.555Z",
      duration: 459,
    },
    {
      id: "4eb5ae13-befb-47cd-8716-5712cefa3676",
      name: "Superman 83",
      videoUrl: "https://exercisedb.org/video/superman-269.mp4",
      form: "good",
      date: "2025-02-08T09:11:08.555Z",
      duration: 536,
    },
    {
      id: "74452010-e2b9-4294-8e62-17b9f632f857",
      name: "Jumping Rope 20",
      videoUrl: "https://workoutlibrary.net/videos/jumping-rope-501.mp4",
      form: "medium",
      date: "2025-02-21T09:11:08.555Z",
      duration: 97,
    },
    {
      id: "a2eaa223-53fa-4090-be6e-b1bae15d3172",
      name: "Wall Sits 1",
      videoUrl: "https://exercisedb.org/video/wall-sits-269.mp4",
      form: "good",
      date: "2025-04-05T08:11:08.555Z",
      duration: 297,
    },
    {
      id: "a0f1e95c-3a96-46a2-9973-efc120a6c53c",
      name: "TRX Rows 79",
      videoUrl: "https://workoutlibrary.net/videos/trx-rows-70.mp4",
      form: "medium",
      date: "2025-02-15T09:11:08.555Z",
      duration: 38,
    },
    {
      id: "a89cc5cb-e72c-4672-8fe2-5c698a0cf4cb",
      name: "Front Raises 76",
      videoUrl: "https://workoutlibrary.net/videos/front-raises-76.mp4",
      form: "bad",
      date: "2025-02-05T09:11:08.555Z",
      duration: 509,
    },
    {
      id: "6b06da3a-ded0-4019-8df4-a12aa52d8e6b",
      name: "Hammer Curls 75",
      videoUrl: "https://trainingmedia.com/exercises/hammer-curls-12.mp4",
      form: "good",
      date: "2025-02-22T09:11:08.555Z",
      duration: 473,
    },
    {
      id: "1fc9c32a-d448-4127-9e89-29e926c09ab0",
      name: "Hammer Curls 68",
      videoUrl: "https://workoutlibrary.net/videos/hammer-curls-160.mp4",
      form: "medium",
      date: "2025-03-27T09:11:08.555Z",
      duration: 358,
    },
    {
      id: "73ed1871-9307-4502-9cae-7c815d8222cb",
      name: "Calf Raises 5",
      videoUrl: "https://trainingmedia.com/exercises/calf-raises-365.mp4",
      form: "good",
      date: "2025-03-09T09:11:08.555Z",
      duration: 568,
    },
    {
      id: "60720eeb-0dd5-428c-969c-3b11637bd779",
      name: "Battle Ropes 3",
      videoUrl: "https://trainingmedia.com/exercises/battle-ropes-773.mp4",
      form: "good",
      date: "2025-02-20T09:11:08.555Z",
      duration: 59,
    },
    {
      id: "a5b55ec5-2333-462f-9e94-a1fe6bb2fe67",
      name: "TRX Rows 66",
      videoUrl: "https://workoutlibrary.net/videos/trx-rows-669.mp4",
      form: "good",
      date: "2025-04-01T08:11:08.555Z",
      duration: 298,
    },
    {
      id: "b87ca7ea-2b64-47ca-9601-60d36c6233ab",
      name: "Box Jumps 39",
      videoUrl: "https://exercisedb.org/video/box-jumps-970.mp4",
      form: "medium",
      date: "2025-03-03T09:11:08.555Z",
      duration: 102,
    },
    {
      id: "8f553090-72f3-495c-8375-d570c5bc12f5",
      name: "Skull Crushers 43",
      videoUrl: "https://workoutlibrary.net/videos/skull-crushers-548.mp4",
      form: "bad",
      date: "2025-02-28T09:11:08.555Z",
      duration: 558,
    },
    {
      id: "2e154098-69e9-4c07-add0-89f795b5cf5a",
      name: "TRX Rows 33",
      videoUrl: "https://fitnessvids.com/exercise-trx-rows-981.mp4",
      form: "bad",
      date: "2025-03-01T09:11:08.555Z",
      duration: 557,
    },
    {
      id: "9c73118d-27aa-4711-bb2e-efe9ebe3c1f8",
      name: "Glute Bridges 94",
      videoUrl: "https://fitnessvids.com/exercise-glute-bridges-580.mp4",
      form: "medium",
      date: "2025-02-06T09:11:08.555Z",
      duration: 452,
    },
    {
      id: "82cef238-80a6-4b75-8541-425670a90985",
      name: "Medicine Ball Throws 20",
      videoUrl:
        "https://trainingmedia.com/exercises/medicine-ball-throws-772.mp4",
      form: "bad",
      date: "2025-02-23T09:11:08.555Z",
      duration: 538,
    },
    {
      id: "344a3054-3fca-4a9b-b3a9-1cca9563d1d1",
      name: "Wall Sits 2",
      videoUrl: "https://fitnessvids.com/exercise-wall-sits-469.mp4",
      form: "medium",
      date: "2025-03-13T09:11:08.555Z",
      duration: 512,
    },
    {
      id: "b99b7360-39b9-4d21-8cd8-e4927619058b",
      name: "Plank 65",
      videoUrl: "https://fitnessvids.com/exercise-plank-38.mp4",
      form: "bad",
      date: "2025-02-19T09:11:08.555Z",
      duration: 79,
    },
    {
      id: "166c0e3a-138f-41b7-8d6d-d952dde2757a",
      name: "Bicep Curls 43",
      videoUrl: "https://trainingmedia.com/exercises/bicep-curls-94.mp4",
      form: "bad",
      date: "2025-02-05T09:11:08.555Z",
      duration: 36,
    },
    {
      id: "d49d69a8-ce98-49a2-a893-fa6eb604ae07",
      name: "Lateral Raises 33",
      videoUrl: "https://exercisedb.org/video/lateral-raises-69.mp4",
      form: "medium",
      date: "2025-04-05T08:11:08.555Z",
      duration: 442,
    },
    {
      id: "f032ee18-5624-4611-8907-9e08c90c1354",
      name: "Squats 66",
      videoUrl: "https://fitnessvids.com/exercise-squats-932.mp4",
      form: "medium",
      date: "2025-03-28T09:11:08.555Z",
      duration: 527,
    },
    {
      id: "3822f661-e9f3-4f75-a330-b3cee4b2e8a8",
      name: "Superman 47",
      videoUrl: "https://trainingmedia.com/exercises/superman-600.mp4",
      form: "good",
      date: "2025-03-20T09:11:08.556Z",
      duration: 433,
    },
    {
      id: "369db0d5-01f8-4405-8df9-7392108124c5",
      name: "Front Raises 67",
      videoUrl: "https://fitnessvids.com/exercise-front-raises-157.mp4",
      form: "bad",
      date: "2025-03-28T09:11:08.556Z",
      duration: 521,
    },
    {
      id: "379ca0eb-5d15-4fcc-9d5b-a418f857cff2",
      name: "Medicine Ball Throws 5",
      videoUrl:
        "https://trainingmedia.com/exercises/medicine-ball-throws-20.mp4",
      form: "medium",
      date: "2025-03-22T09:11:08.556Z",
      duration: 173,
    },
    {
      id: "ca491b43-2c73-4a6e-a3fd-040e91023df0",
      name: "Tricep Extensions 90",
      videoUrl: "https://workoutlibrary.net/videos/tricep-extensions-550.mp4",
      form: "good",
      date: "2025-04-02T08:11:08.556Z",
      duration: 130,
    },
    {
      id: "88e0ce08-d669-458b-aba6-be672d5a3899",
      name: "Glute Bridges 33",
      videoUrl: "https://fitnessvids.com/exercise-glute-bridges-61.mp4",
      form: "good",
      date: "2025-03-04T09:11:08.556Z",
      duration: 477,
    },
    {
      id: "44bfb975-9460-4114-859a-98b62f9ad036",
      name: "Side Planks 36",
      videoUrl: "https://fitnessvids.com/exercise-side-planks-414.mp4",
      form: "medium",
      date: "2025-02-15T09:11:08.556Z",
      duration: 37,
    },
    {
      id: "e0769782-e5fa-42c3-92c4-a5292dce73a7",
      name: "Leg Press 68",
      videoUrl: "https://workoutlibrary.net/videos/leg-press-389.mp4",
      form: "medium",
      date: "2025-03-20T09:11:08.556Z",
      duration: 88,
    },
    {
      id: "ed5d440b-8c31-441a-82b1-23f5b0ce75c5",
      name: "Reverse Flyes 33",
      videoUrl: "https://exercisedb.org/video/reverse-flyes-483.mp4",
      form: "bad",
      date: "2025-03-23T09:11:08.556Z",
      duration: 74,
    },
    {
      id: "ac668e9b-99d8-4e55-831f-6f9c1b634282",
      name: "Dips 82",
      videoUrl: "https://exercisedb.org/video/dips-149.mp4",
      form: "medium",
      date: "2025-03-16T09:11:08.556Z",
      duration: 575,
    },
    {
      id: "3d90d307-ec1e-44bb-b085-41b912c98d1b",
      name: "Squats 25",
      videoUrl: "https://fitnessvids.com/exercise-squats-708.mp4",
      form: "bad",
      date: "2025-03-21T09:11:08.556Z",
      duration: 282,
    },
    {
      id: "3725feeb-1061-4f51-8eda-528d90c1f7dd",
      name: "Tricep Extensions 74",
      videoUrl: "https://exercisedb.org/video/tricep-extensions-967.mp4",
      form: "good",
      date: "2025-04-01T08:11:08.556Z",
      duration: 283,
    },
    {
      id: "ef12e677-a7a5-4863-a58b-84b4a67a80b0",
      name: "Leg Press 84",
      videoUrl: "https://exercisedb.org/video/leg-press-403.mp4",
      form: "medium",
      date: "2025-02-17T09:11:08.556Z",
      duration: 501,
    },
    {
      id: "c2963d57-68da-4b85-b359-8de13f0df3ab",
      name: "High Knees 66",
      videoUrl: "https://trainingmedia.com/exercises/high-knees-14.mp4",
      form: "medium",
      date: "2025-03-07T09:11:08.556Z",
      duration: 84,
    },
    {
      id: "3bafb206-e0fe-4e7b-a7bf-f0f46eec70bd",
      name: "Plank 26",
      videoUrl: "https://exercisedb.org/video/plank-289.mp4",
      form: "bad",
      date: "2025-03-01T09:11:08.556Z",
      duration: 315,
    },
    {
      id: "12b721dd-ab3c-4946-84fb-41d148000b14",
      name: "Reverse Flyes 79",
      videoUrl: "https://fitnessvids.com/exercise-reverse-flyes-667.mp4",
      form: "bad",
      date: "2025-02-22T09:11:08.556Z",
      duration: 416,
    },
    {
      id: "5bd5750b-efda-4689-b8b5-31a0075c93c7",
      name: "Leg Press 52",
      videoUrl: "https://workoutlibrary.net/videos/leg-press-286.mp4",
      form: "medium",
      date: "2025-02-05T09:11:08.556Z",
      duration: 253,
    },
    {
      id: "44c9ac07-7b24-4dfc-9c40-888e0f6e0bf4",
      name: "Russian Twists 35",
      videoUrl: "https://trainingmedia.com/exercises/russian-twists-882.mp4",
      form: "bad",
      date: "2025-03-07T09:11:08.556Z",
      duration: 256,
    },
    {
      id: "ac54078f-1196-4499-a32f-77e571f5695e",
      name: "Cable Pushdowns 9",
      videoUrl: "https://workoutlibrary.net/videos/cable-pushdowns-129.mp4",
      form: "bad",
      date: "2025-03-21T09:11:08.556Z",
      duration: 362,
    },
    {
      id: "4900e70d-b95b-4fc4-9262-7baf97b82ab9",
      name: "Crunches 27",
      videoUrl: "https://workoutlibrary.net/videos/crunches-191.mp4",
      form: "medium",
      date: "2025-02-24T09:11:08.556Z",
      duration: 287,
    },
    {
      id: "a29b396a-9fa3-401a-a8d0-064bd5f4b7bd",
      name: "Squats 55",
      videoUrl: "https://workoutlibrary.net/videos/squats-87.mp4",
      form: "medium",
      date: "2025-02-12T09:11:08.556Z",
      duration: 141,
    },
    {
      id: "2f4b9548-940a-4661-b8d7-238de275e407",
      name: "Hammer Curls 87",
      videoUrl: "https://trainingmedia.com/exercises/hammer-curls-190.mp4",
      form: "good",
      date: "2025-03-07T09:11:08.556Z",
      duration: 35,
    },
    {
      id: "f6397c9c-e7e8-412a-9fb4-7d34205d0aa8",
      name: "TRX Rows 39",
      videoUrl: "https://exercisedb.org/video/trx-rows-134.mp4",
      form: "good",
      date: "2025-03-31T08:11:08.556Z",
      duration: 118,
    },
    {
      id: "92cf50b9-8644-4073-a7b2-ef83549d240a",
      name: "Hip Thrusts 56",
      videoUrl: "https://exercisedb.org/video/hip-thrusts-462.mp4",
      form: "good",
      date: "2025-04-01T08:11:08.556Z",
      duration: 110,
    },
    {
      id: "1b0a37c0-16e7-4c5d-98ff-30d4fa391471",
      name: "Chest Flyes 45",
      videoUrl: "https://exercisedb.org/video/chest-flyes-822.mp4",
      form: "good",
      date: "2025-03-24T09:11:08.556Z",
      duration: 469,
    },
    {
      id: "eb0af9d1-e020-436f-9aa5-976f976cc703",
      name: "Dips 69",
      videoUrl: "https://workoutlibrary.net/videos/dips-661.mp4",
      form: "medium",
      date: "2025-02-16T09:11:08.556Z",
      duration: 363,
    },
    {
      id: "ac84c308-2972-4706-a7f5-b78668177343",
      name: "Box Jumps 59",
      videoUrl: "https://fitnessvids.com/exercise-box-jumps-805.mp4",
      form: "good",
      date: "2025-03-11T09:11:08.556Z",
      duration: 538,
    },
    {
      id: "93dc66b8-0e32-4bba-bcfa-8a05aa991222",
      name: "Leg Press 88",
      videoUrl: "https://fitnessvids.com/exercise-leg-press-392.mp4",
      form: "bad",
      date: "2025-02-07T09:11:08.556Z",
      duration: 444,
    },
    {
      id: "98d4d765-b8f2-4996-9bab-efc5f0b1bb34",
      name: "Wall Sits 42",
      videoUrl: "https://workoutlibrary.net/videos/wall-sits-165.mp4",
      form: "medium",
      date: "2025-03-08T09:11:08.556Z",
      duration: 518,
    },
    {
      id: "747758ba-b222-406c-b289-6a2218c3598c",
      name: "Reverse Flyes 48",
      videoUrl: "https://workoutlibrary.net/videos/reverse-flyes-921.mp4",
      form: "medium",
      date: "2025-03-19T09:11:08.556Z",
      duration: 352,
    },
    {
      id: "0e9e6384-9c51-4562-93ce-b50faa0a55e1",
      name: "Chest Flyes 12",
      videoUrl: "https://trainingmedia.com/exercises/chest-flyes-143.mp4",
      form: "medium",
      date: "2025-02-17T09:11:08.556Z",
      duration: 380,
    },
    {
      id: "8187b941-e218-4fee-9826-092b1b6f6b2f",
      name: "Lat Pulldowns 57",
      videoUrl: "https://fitnessvids.com/exercise-lat-pulldowns-585.mp4",
      form: "bad",
      date: "2025-03-05T09:11:08.556Z",
      duration: 579,
    },
    {
      id: "814976bd-8085-463a-b28f-9a910bbd63d4",
      name: "Pull-ups 85",
      videoUrl: "https://fitnessvids.com/exercise-pull-ups-111.mp4",
      form: "bad",
      date: "2025-03-06T09:11:08.556Z",
      duration: 45,
    },
    {
      id: "1c5ed37e-94f7-4fe0-a037-e120d02ede41",
      name: "Tricep Extensions 22",
      videoUrl: "https://trainingmedia.com/exercises/tricep-extensions-543.mp4",
      form: "medium",
      date: "2025-03-25T09:11:08.556Z",
      duration: 472,
    },
    {
      id: "9e6af2e0-9aeb-4982-8f5d-005ff917e995",
      name: "Tricep Extensions 30",
      videoUrl: "https://workoutlibrary.net/videos/tricep-extensions-468.mp4",
      form: "good",
      date: "2025-03-01T09:11:08.556Z",
      duration: 135,
    },
    {
      id: "19634325-57b3-4ba6-85ad-b50ca75f229c",
      name: "Skull Crushers 58",
      videoUrl: "https://workoutlibrary.net/videos/skull-crushers-806.mp4",
      form: "good",
      date: "2025-03-23T09:11:08.556Z",
      duration: 272,
    },
    {
      id: "964dbcfd-5d0d-459d-bafa-4c01609896f3",
      name: "Skull Crushers 90",
      videoUrl: "https://fitnessvids.com/exercise-skull-crushers-282.mp4",
      form: "bad",
      date: "2025-03-24T09:11:08.556Z",
      duration: 100,
    },
    {
      id: "cfbcf3bb-f62e-4f95-9e63-2b17ccbaeaa6",
      name: "Hip Thrusts 77",
      videoUrl: "https://fitnessvids.com/exercise-hip-thrusts-751.mp4",
      form: "good",
      date: "2025-03-20T09:11:08.556Z",
      duration: 368,
    },
    {
      id: "483e0bda-d77b-484c-a133-0b7583585b69",
      name: "Squats 81",
      videoUrl: "https://trainingmedia.com/exercises/squats-374.mp4",
      form: "good",
      date: "2025-03-12T09:11:08.556Z",
      duration: 88,
    },
    {
      id: "1b046e25-ce47-4adf-8019-341815a66cc1",
      name: "Lateral Raises 62",
      videoUrl: "https://fitnessvids.com/exercise-lateral-raises-843.mp4",
      form: "medium",
      date: "2025-02-19T09:11:08.556Z",
      duration: 440,
    },
    {
      id: "021e9c20-14b5-42f9-b463-70af91ed2ad9",
      name: "Wall Sits 7",
      videoUrl: "https://trainingmedia.com/exercises/wall-sits-679.mp4",
      form: "good",
      date: "2025-03-30T08:11:08.556Z",
      duration: 417,
    },
    {
      id: "533b4c2c-a11d-47dd-8981-9b89e1e6d3d6",
      name: "Shoulder Press 88",
      videoUrl: "https://trainingmedia.com/exercises/shoulder-press-947.mp4",
      form: "bad",
      date: "2025-03-18T09:11:08.556Z",
      duration: 268,
    },
    {
      id: "6c86b8ff-e750-4b65-b722-d165d1e2b54d",
      name: "Deadlifts 19",
      videoUrl: "https://trainingmedia.com/exercises/deadlifts-733.mp4",
      form: "good",
      date: "2025-03-13T09:11:08.556Z",
      duration: 432,
    },
    {
      id: "e1c455d5-d59d-4a10-91c2-dd224eb4102a",
      name: "Burpees 67",
      videoUrl: "https://workoutlibrary.net/videos/burpees-248.mp4",
      form: "bad",
      date: "2025-03-24T09:11:08.556Z",
      duration: 167,
    },
    {
      id: "6d37e788-1ac0-420e-aa6a-8eebf1e0f4ff",
      name: "Step-ups 64",
      videoUrl: "https://exercisedb.org/video/step-ups-852.mp4",
      form: "good",
      date: "2025-03-30T08:11:08.556Z",
      duration: 544,
    },
    {
      id: "56d34542-cff4-4f1f-a15f-9d55da660fc8",
      name: "Bicep Curls 42",
      videoUrl: "https://workoutlibrary.net/videos/bicep-curls-172.mp4",
      form: "good",
      date: "2025-04-02T08:11:08.556Z",
      duration: 307,
    },
    {
      id: "06eac304-ead9-4d82-8a59-a6cd49815934",
      name: "Chest Flyes 62",
      videoUrl: "https://exercisedb.org/video/chest-flyes-343.mp4",
      form: "bad",
      date: "2025-03-27T09:11:08.556Z",
      duration: 580,
    },
    {
      id: "58b4eaab-15a9-4257-bb94-9494064c80c4",
      name: "Deadlifts 87",
      videoUrl: "https://workoutlibrary.net/videos/deadlifts-472.mp4",
      form: "good",
      date: "2025-02-22T09:11:08.556Z",
      duration: 329,
    },
    {
      id: "62bd2dbe-cb91-453a-a5d9-ab0670fddd28",
      name: "Dips 63",
      videoUrl: "https://trainingmedia.com/exercises/dips-457.mp4",
      form: "bad",
      date: "2025-02-13T09:11:08.556Z",
      duration: 479,
    },
    {
      id: "068e16bb-3502-40d5-8d68-b691c883ff6f",
      name: "Battle Ropes 26",
      videoUrl: "https://exercisedb.org/video/battle-ropes-81.mp4",
      form: "medium",
      date: "2025-03-05T09:11:08.556Z",
      duration: 476,
    },
    {
      id: "9d518af6-129c-4d0f-9347-61e14b9dc747",
      name: "Lateral Raises 89",
      videoUrl: "https://fitnessvids.com/exercise-lateral-raises-257.mp4",
      form: "medium",
      date: "2025-02-21T09:11:08.556Z",
      duration: 367,
    },
    {
      id: "685e04dd-6dd8-4308-a573-31adf084dd9c",
      name: "Lat Pulldowns 31",
      videoUrl: "https://exercisedb.org/video/lat-pulldowns-695.mp4",
      form: "bad",
      date: "2025-03-07T09:11:08.556Z",
      duration: 478,
    },
    {
      id: "10903fe0-f124-4c9a-a273-16eb6aa68d82",
      name: "Skull Crushers 44",
      videoUrl: "https://trainingmedia.com/exercises/skull-crushers-72.mp4",
      form: "good",
      date: "2025-04-05T08:11:08.556Z",
      duration: 485,
    },
    {
      id: "b5341577-7222-4e0c-95ca-070d5cd3332d",
      name: "Calf Raises 40",
      videoUrl: "https://exercisedb.org/video/calf-raises-327.mp4",
      form: "bad",
      date: "2025-03-07T09:11:08.556Z",
      duration: 139,
    },
    {
      id: "f9bb18e9-863a-42ad-8029-b5df40307299",
      name: "Face Pulls 84",
      videoUrl: "https://trainingmedia.com/exercises/face-pulls-504.mp4",
      form: "medium",
      date: "2025-03-17T09:11:08.556Z",
      duration: 438,
    },
    {
      id: "c1667e45-9627-4267-9ef2-a10c55355123",
      name: "Plank 83",
      videoUrl: "https://exercisedb.org/video/plank-147.mp4",
      form: "medium",
      date: "2025-03-12T09:11:08.556Z",
      duration: 111,
    },
    {
      id: "771800e2-3878-48e0-9359-27aa4ec5dde8",
      name: "Face Pulls 70",
      videoUrl: "https://trainingmedia.com/exercises/face-pulls-579.mp4",
      form: "good",
      date: "2025-03-10T09:11:08.556Z",
      duration: 169,
    },
    {
      id: "0263baa6-fc2c-455a-b838-74183ef21f88",
      name: "Lat Pulldowns 96",
      videoUrl: "https://fitnessvids.com/exercise-lat-pulldowns-142.mp4",
      form: "medium",
      date: "2025-02-13T09:11:08.556Z",
      duration: 448,
    },
    {
      id: "d9029ac1-60f5-473a-8dca-a12868ea4613",
      name: "Chest Flyes 3",
      videoUrl: "https://exercisedb.org/video/chest-flyes-876.mp4",
      form: "good",
      date: "2025-03-25T09:11:08.556Z",
      duration: 487,
    },
    {
      id: "12640f70-4473-4230-88fb-7106f2ddd7a3",
      name: "Chest Flyes 82",
      videoUrl: "https://workoutlibrary.net/videos/chest-flyes-401.mp4",
      form: "medium",
      date: "2025-02-19T09:11:08.556Z",
      duration: 110,
    },
    {
      id: "6f52a426-6021-46e8-b52f-61fd7ac564bf",
      name: "Deadlifts 93",
      videoUrl: "https://trainingmedia.com/exercises/deadlifts-129.mp4",
      form: "medium",
      date: "2025-02-15T09:11:08.556Z",
      duration: 462,
    },
    {
      id: "12d81c96-f5cc-4197-9872-a62c4b57a864",
      name: "Reverse Flyes 84",
      videoUrl: "https://exercisedb.org/video/reverse-flyes-68.mp4",
      form: "medium",
      date: "2025-02-19T09:11:08.556Z",
      duration: 548,
    },
    {
      id: "4ade6f53-5684-4b0b-8edd-6102f5fae133",
      name: "Shrugs 48",
      videoUrl: "https://trainingmedia.com/exercises/shrugs-125.mp4",
      form: "medium",
      date: "2025-03-15T09:11:08.556Z",
      duration: 264,
    },
    {
      id: "fab8ff0a-09bf-4084-be15-9a4304ae16c9",
      name: "Flutter Kicks 19",
      videoUrl: "https://fitnessvids.com/exercise-flutter-kicks-737.mp4",
      form: "medium",
      date: "2025-03-03T09:11:08.556Z",
      duration: 166,
    },
    {
      id: "1f33c4b5-c46a-4e67-890d-048528b8598f",
      name: "Chest Flyes 53",
      videoUrl: "https://trainingmedia.com/exercises/chest-flyes-679.mp4",
      form: "medium",
      date: "2025-02-07T09:11:08.556Z",
      duration: 89,
    },
    {
      id: "a7e89278-7142-4f5e-bdf7-0cdd09fa7f0d",
      name: "Plank 18",
      videoUrl: "https://trainingmedia.com/exercises/plank-519.mp4",
      form: "medium",
      date: "2025-03-05T09:11:08.556Z",
      duration: 272,
    },
    {
      id: "0b410cc2-9b9c-4902-900e-482851fca4a0",
      name: "Dips 32",
      videoUrl: "https://trainingmedia.com/exercises/dips-268.mp4",
      form: "good",
      date: "2025-03-19T09:11:08.556Z",
      duration: 544,
    },
    {
      id: "606734a2-b64e-4f45-b611-b009362e71fa",
      name: "Hammer Curls 14",
      videoUrl: "https://trainingmedia.com/exercises/hammer-curls-858.mp4",
      form: "medium",
      date: "2025-03-03T09:11:08.556Z",
      duration: 287,
    },
    {
      id: "18473e1c-7c69-4a91-ace9-a0677282b886",
      name: "Step-ups 87",
      videoUrl: "https://trainingmedia.com/exercises/step-ups-843.mp4",
      form: "good",
      date: "2025-03-12T09:11:08.556Z",
      duration: 113,
    },
    {
      id: "b77a66d8-0089-49ed-9801-b9d71f25b781",
      name: "Hip Thrusts 6",
      videoUrl: "https://trainingmedia.com/exercises/hip-thrusts-139.mp4",
      form: "bad",
      date: "2025-03-11T09:11:08.556Z",
      duration: 244,
    },
    {
      id: "fab52e08-204e-4a39-940a-b965212ee33a",
      name: "Front Raises 82",
      videoUrl: "https://fitnessvids.com/exercise-front-raises-192.mp4",
      form: "medium",
      date: "2025-03-31T08:11:08.556Z",
      duration: 442,
    },
    {
      id: "e80e8bfd-6be5-48cb-8bc6-7b0a0fa17279",
      name: "Lat Pulldowns 21",
      videoUrl: "https://exercisedb.org/video/lat-pulldowns-940.mp4",
      form: "bad",
      date: "2025-02-10T09:11:08.556Z",
      duration: 410,
    },
    {
      id: "f881acea-3c33-40b4-8e21-dfb81332848b",
      name: "Push-ups 24",
      videoUrl: "https://fitnessvids.com/exercise-push-ups-323.mp4",
      form: "good",
      date: "2025-03-09T09:11:08.556Z",
      duration: 50,
    },
    {
      id: "dafc89b1-ffb8-49dc-8486-8957b248909a",
      name: "Kettlebell Swings 58",
      videoUrl: "https://trainingmedia.com/exercises/kettlebell-swings-848.mp4",
      form: "bad",
      date: "2025-03-26T09:11:08.556Z",
      duration: 216,
    },
    {
      id: "db0dac74-ec5a-44e4-beee-9a959faf3a09",
      name: "Cable Pushdowns 58",
      videoUrl: "https://fitnessvids.com/exercise-cable-pushdowns-251.mp4",
      form: "bad",
      date: "2025-03-17T09:11:08.556Z",
      duration: 364,
    },
    {
      id: "a10af70c-e42e-494f-b1af-57c32d347bac",
      name: "Hammer Curls 97",
      videoUrl: "https://fitnessvids.com/exercise-hammer-curls-749.mp4",
      form: "bad",
      date: "2025-03-28T09:11:08.556Z",
      duration: 82,
    },
    {
      id: "0c1982ea-b411-4ea9-8e7b-4cf8bff0a285",
      name: "Jumping Jacks 99",
      videoUrl: "https://workoutlibrary.net/videos/jumping-jacks-125.mp4",
      form: "good",
      date: "2025-02-22T09:11:08.556Z",
      duration: 584,
    },
    {
      id: "f6066658-8708-4cb8-8c29-3369f5d18cd9",
      name: "Superman 88",
      videoUrl: "https://trainingmedia.com/exercises/superman-534.mp4",
      form: "medium",
      date: "2025-03-18T09:11:08.556Z",
      duration: 447,
    },
    {
      id: "e6cb9a32-847e-49e5-8b21-5fd425a84691",
      name: "Reverse Flyes 99",
      videoUrl: "https://workoutlibrary.net/videos/reverse-flyes-393.mp4",
      form: "good",
      date: "2025-03-09T09:11:08.556Z",
      duration: 582,
    },
    {
      id: "38f61eed-f792-49ae-8a53-03424a969f79",
      name: "Shoulder Press 25",
      videoUrl: "https://exercisedb.org/video/shoulder-press-140.mp4",
      form: "good",
      date: "2025-02-23T09:11:08.556Z",
      duration: 494,
    },
    {
      id: "7c086d5d-6d84-4ab9-a078-7dc870808e62",
      name: "Box Jumps 34",
      videoUrl: "https://trainingmedia.com/exercises/box-jumps-507.mp4",
      form: "medium",
      date: "2025-03-23T09:11:08.556Z",
      duration: 500,
    },
    {
      id: "91d18e21-4ab3-40a8-8c8d-c003f8b45465",
      name: "High Knees 60",
      videoUrl: "https://exercisedb.org/video/high-knees-18.mp4",
      form: "medium",
      date: "2025-03-19T09:11:08.556Z",
      duration: 597,
    },
    {
      id: "1def8f84-8cdd-4383-a8a0-dc1e38b3a68a",
      name: "Lateral Raises 78",
      videoUrl: "https://fitnessvids.com/exercise-lateral-raises-98.mp4",
      form: "medium",
      date: "2025-02-23T09:11:08.556Z",
      duration: 486,
    },
    {
      id: "fb3956d4-703c-494e-bba3-1e4d7f93de78",
      name: "Skull Crushers 88",
      videoUrl: "https://exercisedb.org/video/skull-crushers-225.mp4",
      form: "medium",
      date: "2025-04-02T08:11:08.556Z",
      duration: 260,
    },
    {
      id: "15fbac0b-fb80-4d9d-889c-08a873f9a10f",
      name: "Deadlifts 68",
      videoUrl: "https://workoutlibrary.net/videos/deadlifts-54.mp4",
      form: "medium",
      date: "2025-03-19T09:11:08.556Z",
      duration: 379,
    },
    {
      id: "2b63e8f7-b3fd-44bd-a8b1-888ae933f474",
      name: "Plank 5",
      videoUrl: "https://trainingmedia.com/exercises/plank-672.mp4",
      form: "good",
      date: "2025-02-12T09:11:08.556Z",
      duration: 522,
    },
    {
      id: "b503a334-0ceb-477f-af40-8c69bf6cf79a",
      name: "Push-ups 22",
      videoUrl: "https://fitnessvids.com/exercise-push-ups-41.mp4",
      form: "medium",
      date: "2025-04-03T08:11:08.556Z",
      duration: 561,
    },
    {
      id: "ad94d19a-2273-43fc-b3da-876dcc62da27",
      name: "Squats 34",
      videoUrl: "https://exercisedb.org/video/squats-827.mp4",
      form: "medium",
      date: "2025-03-07T09:11:08.556Z",
      duration: 248,
    },
    {
      id: "e98af828-90af-4544-bc39-94a5fbfe24e0",
      name: "Calf Raises 81",
      videoUrl: "https://fitnessvids.com/exercise-calf-raises-633.mp4",
      form: "bad",
      date: "2025-02-12T09:11:08.556Z",
      duration: 548,
    },
    {
      id: "56e9652d-c605-4001-a83c-0f57a98bdd6c",
      name: "Flutter Kicks 50",
      videoUrl: "https://workoutlibrary.net/videos/flutter-kicks-969.mp4",
      form: "bad",
      date: "2025-03-10T09:11:08.556Z",
      duration: 141,
    },
    {
      id: "ebe899b8-9d25-4a2e-aee2-712dae567252",
      name: "Reverse Flyes 90",
      videoUrl: "https://trainingmedia.com/exercises/reverse-flyes-257.mp4",
      form: "good",
      date: "2025-03-15T09:11:08.556Z",
      duration: 361,
    },
    {
      id: "cfba66ac-1dfb-4852-948a-37908abf8fcf",
      name: "Cable Pushdowns 68",
      videoUrl: "https://exercisedb.org/video/cable-pushdowns-643.mp4",
      form: "bad",
      date: "2025-03-27T09:11:08.556Z",
      duration: 587,
    },
    {
      id: "ab88eb65-5054-4d55-8684-f9ad37b1c9af",
      name: "High Knees 99",
      videoUrl: "https://trainingmedia.com/exercises/high-knees-519.mp4",
      form: "bad",
      date: "2025-02-28T09:11:08.556Z",
      duration: 318,
    },
    {
      id: "cf1f2a41-cb94-4fc8-84db-f39329bee137",
      name: "High Knees 63",
      videoUrl: "https://exercisedb.org/video/high-knees-658.mp4",
      form: "good",
      date: "2025-02-18T09:11:08.556Z",
      duration: 431,
    },
    {
      id: "6eaed9db-739a-4fc1-b98f-26fede1bf852",
      name: "Lateral Raises 22",
      videoUrl: "https://fitnessvids.com/exercise-lateral-raises-62.mp4",
      form: "medium",
      date: "2025-03-08T09:11:08.556Z",
      duration: 88,
    },
    {
      id: "8cc24d32-16f3-4498-8b83-6a963de6455f",
      name: "Mountain Climbers 35",
      videoUrl: "https://exercisedb.org/video/mountain-climbers-673.mp4",
      form: "bad",
      date: "2025-03-31T08:11:08.556Z",
      duration: 562,
    },
    {
      id: "5c03796f-e2a1-4d5a-a59a-7a0cc044b98f",
      name: "Flutter Kicks 79",
      videoUrl: "https://exercisedb.org/video/flutter-kicks-945.mp4",
      form: "good",
      date: "2025-02-11T09:11:08.556Z",
      duration: 506,
    },
    {
      id: "48c09f0b-2cba-4d25-b631-f7f4e8ce5362",
      name: "Superman 86",
      videoUrl: "https://workoutlibrary.net/videos/superman-586.mp4",
      form: "medium",
      date: "2025-02-15T09:11:08.556Z",
      duration: 367,
    },
    {
      id: "bd96bcde-f60c-42aa-8b14-7749d283c02c",
      name: "Skull Crushers 83",
      videoUrl: "https://fitnessvids.com/exercise-skull-crushers-666.mp4",
      form: "good",
      date: "2025-03-12T09:11:08.556Z",
      duration: 541,
    },
    {
      id: "0315e76f-50cc-4be9-8b2a-f0a7121d1002",
      name: "Push-ups 19",
      videoUrl: "https://exercisedb.org/video/push-ups-584.mp4",
      form: "medium",
      date: "2025-02-23T09:11:08.556Z",
      duration: 146,
    },
    {
      id: "8f0df2be-20e0-4678-81ed-4096be8ad716",
      name: "Calf Raises 10",
      videoUrl: "https://trainingmedia.com/exercises/calf-raises-230.mp4",
      form: "medium",
      date: "2025-03-30T08:11:08.556Z",
      duration: 494,
    },
    {
      id: "44948c4f-8396-429a-9b95-2b7df68fe15f",
      name: "Lunges 1",
      videoUrl: "https://exercisedb.org/video/lunges-782.mp4",
      form: "bad",
      date: "2025-03-01T09:11:08.556Z",
      duration: 498,
    },
    {
      id: "90044e5e-c8ae-4f97-a40c-e55dd3ef571a",
      name: "Battle Ropes 98",
      videoUrl: "https://workoutlibrary.net/videos/battle-ropes-661.mp4",
      form: "good",
      date: "2025-03-28T09:11:08.556Z",
      duration: 354,
    },
    {
      id: "5644b67c-46ac-4d3d-884b-b05cdee3f415",
      name: "Lunges 9",
      videoUrl: "https://exercisedb.org/video/lunges-847.mp4",
      form: "good",
      date: "2025-02-10T09:11:08.556Z",
      duration: 200,
    },
    {
      id: "a583ba9e-07b8-4246-822e-51e832965942",
      name: "High Knees 74",
      videoUrl: "https://fitnessvids.com/exercise-high-knees-28.mp4",
      form: "bad",
      date: "2025-02-05T09:11:08.556Z",
      duration: 537,
    },
    {
      id: "c410003b-1155-4ee5-bff8-90edd5295eb8",
      name: "Calf Raises 79",
      videoUrl: "https://fitnessvids.com/exercise-calf-raises-545.mp4",
      form: "medium",
      date: "2025-03-13T09:11:08.556Z",
      duration: 73,
    },
    {
      id: "3b15afe7-e6ed-4f13-9023-37fb99f724d6",
      name: "Front Raises 14",
      videoUrl: "https://exercisedb.org/video/front-raises-54.mp4",
      form: "good",
      date: "2025-03-25T09:11:08.556Z",
      duration: 227,
    },
    {
      id: "e2bd1e39-ecb6-4075-94fd-81385d600847",
      name: "Side Planks 56",
      videoUrl: "https://trainingmedia.com/exercises/side-planks-806.mp4",
      form: "medium",
      date: "2025-02-28T09:11:08.556Z",
      duration: 262,
    },
    {
      id: "59545a5b-d34b-4085-aa37-b3289dae13c1",
      name: "Cable Pushdowns 23",
      videoUrl: "https://exercisedb.org/video/cable-pushdowns-699.mp4",
      form: "good",
      date: "2025-02-06T09:11:08.556Z",
      duration: 454,
    },
    {
      id: "57f9e472-b583-4352-b4a2-2bd5db3bf0d1",
      name: "Reverse Flyes 39",
      videoUrl: "https://fitnessvids.com/exercise-reverse-flyes-638.mp4",
      form: "medium",
      date: "2025-02-08T09:11:08.556Z",
      duration: 143,
    },
    {
      id: "d14b0d27-cc7d-46d0-b9f5-acbeb45afe91",
      name: "Mountain Climbers 3",
      videoUrl: "https://fitnessvids.com/exercise-mountain-climbers-275.mp4",
      form: "good",
      date: "2025-03-24T09:11:08.556Z",
      duration: 431,
    },
    {
      id: "ce310b14-9af3-4785-aa90-7c159ce8e7f2",
      name: "TRX Rows 32",
      videoUrl: "https://fitnessvids.com/exercise-trx-rows-240.mp4",
      form: "bad",
      date: "2025-03-04T09:11:08.556Z",
      duration: 589,
    },
    {
      id: "d592a7f6-9f9b-4d56-a2f8-af8e910c6fba",
      name: "Box Jumps 41",
      videoUrl: "https://fitnessvids.com/exercise-box-jumps-584.mp4",
      form: "good",
      date: "2025-03-27T09:11:08.556Z",
      duration: 378,
    },
    {
      id: "9b68513c-53e4-42bd-b945-fb0bf47a941d",
      name: "Kettlebell Swings 76",
      videoUrl: "https://workoutlibrary.net/videos/kettlebell-swings-218.mp4",
      form: "medium",
      date: "2025-03-22T09:11:08.556Z",
      duration: 508,
    },
    {
      id: "e8772260-8447-44df-bc35-942f73901657",
      name: "Flutter Kicks 91",
      videoUrl: "https://workoutlibrary.net/videos/flutter-kicks-164.mp4",
      form: "bad",
      date: "2025-03-20T09:11:08.556Z",
      duration: 215,
    },
    {
      id: "14eb9ca3-c1d6-416b-baf0-2aedda40050a",
      name: "Push-ups 32",
      videoUrl: "https://fitnessvids.com/exercise-push-ups-503.mp4",
      form: "bad",
      date: "2025-03-09T09:11:08.556Z",
      duration: 83,
    },
    {
      id: "edf906a5-a441-41e0-af69-06c607226300",
      name: "Crunches 54",
      videoUrl: "https://exercisedb.org/video/crunches-212.mp4",
      form: "medium",
      date: "2025-03-20T09:11:08.556Z",
      duration: 455,
    },
    {
      id: "68b256cd-fd56-4081-ab5a-7d3ed187f1da",
      name: "Push-ups 5",
      videoUrl: "https://trainingmedia.com/exercises/push-ups-393.mp4",
      form: "good",
      date: "2025-02-19T09:11:08.556Z",
      duration: 221,
    },
    {
      id: "a802c2b1-d003-44ea-9746-9cfaefddc88f",
      name: "Leg Press 87",
      videoUrl: "https://workoutlibrary.net/videos/leg-press-593.mp4",
      form: "good",
      date: "2025-03-27T09:11:08.556Z",
      duration: 139,
    },
    {
      id: "f8e5c71b-edae-4d24-abf5-873e73ad6c7b",
      name: "Preacher Curls 58",
      videoUrl: "https://fitnessvids.com/exercise-preacher-curls-134.mp4",
      form: "medium",
      date: "2025-02-25T09:11:08.556Z",
      duration: 565,
    },
    {
      id: "77c34968-8fd6-4d34-80d3-16a4b1a763dd",
      name: "Dips 83",
      videoUrl: "https://workoutlibrary.net/videos/dips-73.mp4",
      form: "bad",
      date: "2025-03-24T09:11:08.556Z",
      duration: 335,
    },
    {
      id: "b013e55d-c031-4d4c-9074-43406f1c2d93",
      name: "Wall Sits 18",
      videoUrl: "https://fitnessvids.com/exercise-wall-sits-14.mp4",
      form: "good",
      date: "2025-02-28T09:11:08.556Z",
      duration: 125,
    },
  ];

  getAll(): Exercise[] {
    return this.exercises;
  }

  getById(id: string): Exercise | undefined {
    console.log(`ExerciseStore.getById called with ID: ${id}`);

    // Normalize the input ID (handle both string and numeric formats)
    const normalizedInputId = String(id).trim();
    console.log(`Normalized input ID: ${normalizedInputId}`);

    let exercise = this.exercises.find((ex) => ex.id === normalizedInputId);

    if (!exercise && /^\d+$/.test(normalizedInputId)) {
      console.log(`Trying numeric comparison for ID: ${normalizedInputId}`);
      exercise = this.exercises.find((ex) => {
        // Normalize stored ID if it's numeric
        if (/^\d+$/.test(ex.id)) {
          const normalizedStoredId = String(ex.id).trim();
          const match = normalizedStoredId === normalizedInputId;
          if (match) {
            console.log(
              `Found match with normalized ID comparison: ${ex.id} === ${normalizedInputId}`,
            );
          }
          return match;
        }
        return false;
      });
    }

    console.log(
      `ExerciseStore.getById result for ID ${id}:`,
      exercise ? "Found" : "Not Found",
    );

    // If we still didn't find it, log all IDs for debugging
    if (!exercise) {
      const allIds = this.exercises.map((ex) => ex.id);
      console.log(`Available exercise IDs:`, allIds);
    }

    return exercise;
  }

  add(exercise: Omit<Exercise, "id">): Exercise {
    const newExercise: Exercise = {
      ...exercise,
      id: Date.now().toString(),
    };

    console.log(
      `ExerciseStore.add: Creating new exercise with ID ${newExercise.id}`,
    );
    this.exercises = [newExercise, ...this.exercises];
    return newExercise;
  }

  update(id: string, updates: Partial<Omit<Exercise, "id">>): Exercise | null {
    console.log(
      `ExerciseStore.update: Updating exercise with ID ${id}`,
      updates,
    );

    // Normalize the input ID (handle both string and numeric formats)
    const normalizedInputId = String(id).trim();
    console.log(`Normalized input ID for update: ${normalizedInputId}`);

    // Try direct match first
    let index = this.exercises.findIndex((ex) => ex.id === normalizedInputId);

    // If not found, try numeric comparison if the ID looks like a number
    if (index === -1 && /^\d+$/.test(normalizedInputId)) {
      console.log(
        `Trying numeric comparison for update ID: ${normalizedInputId}`,
      );
      index = this.exercises.findIndex((ex) => {
        // Normalize stored ID if it's numeric
        if (/^\d+$/.test(ex.id)) {
          const normalizedStoredId = String(ex.id).trim();
          const match = normalizedStoredId === normalizedInputId;
          if (match) {
            console.log(
              `Found match for update with normalized ID comparison: ${ex.id} === ${normalizedInputId}`,
            );
          }
          return match;
        }
        return false;
      });
    }

    if (index === -1) {
      console.error(`Exercise with ID ${id} not found for update operation`);
      // Debug: list IDs of all exercises to help diagnose the issue
      const allIds = this.exercises.map((ex) => ex.id);
      console.log(`Available exercise IDs:`, allIds);
      return null;
    }

    const updatedExercise = {
      ...this.exercises[index],
      ...updates,
    };

    this.exercises[index] = updatedExercise;
    console.log(
      `ExerciseStore.update: Successfully updated exercise with ID ${id}`,
    );
    return updatedExercise;
  }

  delete(id: string): Exercise | null {
    console.log(`ExerciseStore.delete: Deleting exercise with ID ${id}`);

    // Normalize the input ID (handle both string and numeric formats)
    const normalizedInputId = String(id).trim();
    console.log(`Normalized input ID for delete: ${normalizedInputId}`);

    // Try direct match first
    let index = this.exercises.findIndex((ex) => ex.id === normalizedInputId);

    // If not found, try numeric comparison if the ID looks like a number
    if (index === -1 && /^\d+$/.test(normalizedInputId)) {
      console.log(
        `Trying numeric comparison for delete ID: ${normalizedInputId}`,
      );
      index = this.exercises.findIndex((ex) => {
        // Normalize stored ID if it's numeric
        if (/^\d+$/.test(ex.id)) {
          const normalizedStoredId = String(ex.id).trim();
          const match = normalizedStoredId === normalizedInputId;
          if (match) {
            console.log(
              `Found match for delete with normalized ID comparison: ${ex.id} === ${normalizedInputId}`,
            );
          }
          return match;
        }
        return false;
      });
    }

    if (index === -1) {
      console.error(`Exercise with ID ${id} not found for delete operation`);
      // Debug: list IDs of all exercises to help diagnose the issue
      const allIds = this.exercises.map((ex) => ex.id);
      console.log(`Available exercise IDs:`, allIds);
      return null;
    }

    const [deletedExercise] = this.exercises.splice(index, 1);
    console.log(
      `ExerciseStore.delete: Successfully deleted exercise with ID ${id}`,
    );
    return deletedExercise;
  }

  filter(criteria: Partial<Exercise>): Exercise[] {
    return this.exercises.filter((exercise) => {
      return Object.entries(criteria).every(([key, value]) => {
        return exercise[key as keyof Exercise] === value;
      });
    });
  }
}

export const exerciseStore = new ExerciseStore();
