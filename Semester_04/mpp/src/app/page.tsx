import Camera from "./_components/Camera";
import PageLayout from "./_components/PageLayout";
import ExerciseGeneratorControl from "./_components/ExerciseGeneratorControl";

export default function Home() {
  return (
    <PageLayout>
      <div className="flex h-[calc(100vh-4rem)] flex-col px-4">
        <div className="mx-auto flex h-full w-full max-w-md items-center justify-center">
          <Camera />
        </div>

        <div className="mx-auto mb-8 w-full max-w-md">
          <ExerciseGeneratorControl />
        </div>
      </div>
    </PageLayout>
  );
}
