import { z } from "zod";

export const resultAnnotationSchema = z.object({
  label: z.string().min(1),
  start_time: z.number().finite().nonnegative(),
  end_time: z.number().finite().nonnegative()
}).refine((annotation) => annotation.end_time >= annotation.start_time, {
  message: "Annotation end_time must be greater than or equal to start_time."
});

export const resultEpisodeSchema = z.object({
  filename: z.string().min(1),
  annotations: z.array(resultAnnotationSchema)
});

export const uploadResultSchema = z.object({
  episodes: z.array(resultEpisodeSchema).min(1)
});

export type ResultAnnotation = z.infer<typeof resultAnnotationSchema>;
export type ResultEpisode = z.infer<typeof resultEpisodeSchema>;
export type UploadResult = z.infer<typeof uploadResultSchema>;

export function parseUploadResultJson(value: string): UploadResult {
  return uploadResultSchema.parse(JSON.parse(value));
}
