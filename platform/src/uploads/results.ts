import { z } from "zod";

export const resultAnnotationSchema = z.object({
  label: z.string().min(1),
  timestamp: z.number().finite().nonnegative()
});

export const resultEpisodeSchema = z.object({
  episode_id: z.string().min(1),
  annotations: z.array(resultAnnotationSchema)
});

export const uploadResultSchema = z.object({
  episodes: z.array(resultEpisodeSchema).min(1)
});

export type ResultAnnotation = z.infer<typeof resultAnnotationSchema>;
export type ResultEpisode = z.infer<typeof resultEpisodeSchema>;
export type UploadResult = z.infer<typeof uploadResultSchema>;

export function parseUploadResultJson(value: string): UploadResult {
  const parsed = JSON.parse(value);

  // Keep older generated results readable while using the end-timestamp format
  // for all new result files.
  if (Array.isArray(parsed?.episodes) && parsed.episodes.every((episode: unknown) => {
    if (!episode || typeof episode !== "object") return false;
    const annotations = (episode as { annotations?: unknown }).annotations;
    return Array.isArray(annotations) && annotations.every((annotation: unknown) => {
      if (!annotation || typeof annotation !== "object") return false;
      return "start_time" in annotation || "end_time" in annotation;
    });
  })) {
    return uploadResultSchema.parse({
      episodes: parsed.episodes.map((episode: { filename: string; annotations: Array<{ label: string; end_time: number }> }) => ({
        episode_id: episode.filename,
        annotations: episode.annotations.map((annotation) => ({
          label: annotation.label,
          timestamp: annotation.end_time
        }))
      }))
    });
  }

  return uploadResultSchema.parse(parsed);
}
