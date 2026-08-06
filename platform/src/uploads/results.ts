import { z } from "zod";

export const resultAnnotationSchema = z.object({
  label: z.string().min(1),
  start_time: z.number().finite().nonnegative(),
  end_time: z.number().finite().nonnegative()
}).refine((annotation) => annotation.end_time >= annotation.start_time, {
  message: "Annotation end_time must be greater than or equal to start_time."
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

  const raw = z.object({
    episodes: z.array(z.object({
      episode_id: z.string().min(1).optional(),
      filename: z.string().min(1).optional(),
      annotations: z.array(z.object({
        label: z.string().min(1),
        timestamp: z.number().finite().nonnegative().optional(),
        start_time: z.number().finite().nonnegative().optional(),
        end_time: z.number().finite().nonnegative().optional()
      }))
    })).min(1)
  }).parse(parsed);

  return uploadResultSchema.parse({
    episodes: raw.episodes.map((episode) => {
      const episodeId = episode.episode_id ?? episode.filename;
      if (!episodeId) {
        throw new Error("Each result episode needs an episode_id.");
      }

      let previousEnd = 0;
      const annotations = episode.annotations.map((annotation) => {
        const endTime = annotation.timestamp ?? annotation.end_time ?? annotation.start_time;
        if (endTime === undefined) {
          throw new Error("Each result annotation needs a timestamp or end_time.");
        }

        const normalized = {
          label: annotation.label,
          start_time: previousEnd,
          end_time: endTime
        };
        previousEnd = endTime;
        return normalized;
      });

      return { episode_id: episodeId, annotations };
    })
  });
}
