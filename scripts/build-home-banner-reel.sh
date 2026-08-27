#!/usr/bin/env bash

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(cd -- "$script_dir/.." && pwd)
source_dir="$repo_dir/_source_media/home-banner"
video_dir="$repo_dir/assets/videos"
image_dir="$repo_dir/assets/img"
work_dir=$(mktemp -d /tmp/home-banner-reel.XXXXXX)

segment_duration="2.500"
transition_duration="0.375"
frame_rate="24"
frame_width="1600"
frame_height="720"
frame_size="1600x720"
portrait_height="600"
portrait_stage_width="1080"
portrait_stage_height="600"
wide_stage_width="1280"
edge_feather="48"

cleanup() {
  case "$work_dir" in
    /tmp/home-banner-reel.*) rm -rf -- "$work_dir" ;;
  esac
}
trap cleanup EXIT

mkdir -p -- "$video_dir" "$image_dir"

require_source() {
  if [[ ! -f "$source_dir/$1" ]]; then
    printf 'Missing banner source: %s\n' "$source_dir/$1" >&2
    exit 1
  fi
}

for source_name in \
  humanoid-robot-box-transfer-demo.mp4 \
  humanoid-teleoperation-news-clip.mp4 \
  indoor-drone-flight-test.mov \
  turtlebot3-vision-guided-grasping-demo.mp4 \
  ai-assisted-pcb-inspection-demo.mp4 \
  robot-assisted-crack-detection-demo.mp4 \
  3d-printer-fabrication-demo.mov \
  automated-catalyst-cleaning-robot-components.jpg \
  automated-catalyst-cleaning-frame-grinding.mp4 \
  automated-catalyst-cleaning-robot-rail-test.mp4 \
  automated-mixing-system-agitation-test.mp4; do
  require_source "$source_name"
done

encode_landscape_video() {
  local input_path="$1"
  local output_path="$2"
  local target_duration="${3:-$segment_duration}"
  local keep_fraction="${4:-1.0}"
  local crop_anchor_y="${5:-0.5}"
  local source_start="${6:-0.0}"
  local content_width="${7:-$frame_width}"
  local source_duration
  local available_duration
  local trimmed_duration
  local pts_factor

  source_duration=$(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$input_path")
  available_duration=$(awk -v source="$source_duration" -v start="$source_start" 'BEGIN { printf "%.10f", source - start }')
  trimmed_duration=$(awk -v available="$available_duration" -v keep="$keep_fraction" 'BEGIN { printf "%.10f", available * keep }')
  pts_factor=$(awk -v target="$target_duration" -v source="$trimmed_duration" 'BEGIN { printf "%.10f", target / source }')

  if [[ "$content_width" == "$frame_width" ]]; then
    ffmpeg -nostdin -hide_banner -loglevel error -y \
      -i "$input_path" \
      -an \
      -vf "trim=start=${source_start}:duration=${trimmed_duration},setpts=(PTS-STARTPTS)*${pts_factor},fps=${frame_rate},scale=${frame_width}:${frame_height}:force_original_aspect_ratio=increase:flags=lanczos,crop=${frame_width}:${frame_height}:(in_w-out_w)/2:(in_h-out_h)*${crop_anchor_y},setsar=1,format=yuv420p" \
      -t "$target_duration" \
      -c:v libx264 -preset fast -crf 18 -video_track_timescale 24000 \
      "$output_path"
  else
    ffmpeg -nostdin -hide_banner -loglevel error -y \
      -i "$input_path" \
      -an \
      -filter_complex "color=c=black:s=${frame_size}:r=${frame_rate}:d=${target_duration}[bg-ready];[0:v]trim=start=${source_start}:duration=${trimmed_duration},setpts=(PTS-STARTPTS)*${pts_factor},fps=${frame_rate},scale=${content_width}:${frame_height}:force_original_aspect_ratio=increase:flags=lanczos,crop=${content_width}:${frame_height}:(in_w-out_w)/2:(in_h-out_h)*${crop_anchor_y},format=yuv444p,split=2[fg-color][fg-mask-source];[fg-mask-source]format=gray,geq=lum='clip(255*X/${edge_feather},0,255)'[fg-mask];[fg-color][fg-mask]alphamerge[fg-card];[bg-ready][fg-card]overlay=W-w:0,setsar=1,format=yuv420p[out]" \
      -map '[out]' \
      -t "$target_duration" \
      -c:v libx264 -preset fast -crf 18 -video_track_timescale 24000 \
      "$output_path"
  fi
}

encode_contained_video() {
  local input_path="$1"
  local output_path="$2"
  local target_duration="${3:-$segment_duration}"
  local keep_fraction="${4:-1.0}"
  local crop_top_fraction="${5:-0.0}"
  local crop_bottom_fraction="${6:-0.0}"
  local source_start="${7:-0.0}"
  local content_height="${8:-$portrait_height}"
  local fill_stage="${9:-false}"
  local source_clip_duration="${10:-}"
  local source_duration
  local available_duration
  local trimmed_duration
  local crop_height_fraction
  local pts_factor
  local foreground_scale

  source_duration=$(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$input_path")
  available_duration=$(awk -v source="$source_duration" -v start="$source_start" 'BEGIN { printf "%.10f", source - start }')
  if [[ -n "$source_clip_duration" ]]; then
    trimmed_duration="$source_clip_duration"
  else
    trimmed_duration=$(awk -v available="$available_duration" -v keep="$keep_fraction" 'BEGIN { printf "%.10f", available * keep }')
  fi
  crop_height_fraction=$(awk -v top="$crop_top_fraction" -v bottom="$crop_bottom_fraction" 'BEGIN { printf "%.10f", 1 - top - bottom }')
  pts_factor=$(awk -v target="$target_duration" -v source="$trimmed_duration" 'BEGIN { printf "%.10f", target / source }')

  if [[ "$fill_stage" == "true" ]]; then
    foreground_scale="scale=${portrait_stage_width}:${portrait_stage_height}:force_original_aspect_ratio=increase:flags=lanczos,crop=${portrait_stage_width}:${portrait_stage_height}:(in_w-out_w)/2:(in_h-out_h)/2"
  else
    foreground_scale="scale=${frame_width}:${content_height}:force_original_aspect_ratio=decrease:flags=lanczos"
  fi

  ffmpeg -nostdin -hide_banner -loglevel error -y \
    -i "$input_path" \
    -an \
    -filter_complex "color=c=black:s=${frame_size}:r=${frame_rate}:d=${target_duration}[bg-ready];[0:v]trim=start=${source_start}:duration=${trimmed_duration},setpts=(PTS-STARTPTS)*${pts_factor},crop=iw:trunc(ih*${crop_height_fraction}/2)*2:0:trunc(ih*${crop_top_fraction}/2)*2,fps=${frame_rate},${foreground_scale},eq=contrast=1.04:saturation=1.04,format=yuv444p,split=2[fg-color][fg-mask-source];[fg-mask-source]format=gray,geq=lum='clip(255*X/${edge_feather},0,255)'[fg-mask];[fg-color][fg-mask]alphamerge[fg-card];[bg-ready][fg-card]overlay=W-w:(H-h)/2,setsar=1,format=yuv420p[out]" \
    -map '[out]' \
    -t "$target_duration" \
    -c:v libx264 -preset fast -crf 18 -video_track_timescale 24000 \
    "$output_path"
}

encode_photo_card() {
  local input_path="$1"
  local output_path="$2"
  local crop_top_fraction="${3:-0.0}"
  local crop_bottom_fraction="${4:-0.0}"
  local crop_height_fraction

  crop_height_fraction=$(awk -v top="$crop_top_fraction" -v bottom="$crop_bottom_fraction" 'BEGIN { printf "%.10f", 1 - top - bottom }')

  ffmpeg -nostdin -hide_banner -loglevel error -y \
    -i "$input_path" \
    -an \
    -filter_complex "color=c=black:s=${frame_size}:r=${frame_rate}:d=${segment_duration}[bg-ready];[0:v]transpose=clock,crop=iw:trunc(ih*${crop_height_fraction}/2)*2:0:trunc(ih*${crop_top_fraction}/2)*2,scale=${portrait_stage_width}:${portrait_stage_height}:force_original_aspect_ratio=increase:flags=lanczos,crop=${portrait_stage_width}:${portrait_stage_height}:(in_w-out_w)/2:(in_h-out_h)/2,eq=contrast=1.04:saturation=1.04,tpad=stop_mode=clone:stop_duration=${segment_duration},fps=${frame_rate},setpts=PTS-STARTPTS,format=yuv444p,split=2[fg-color][fg-mask-source];[fg-mask-source]format=gray,geq=lum='clip(255*X/${edge_feather},0,255)'[fg-mask];[fg-color][fg-mask]alphamerge[fg-feather];[bg-ready][fg-feather]overlay=x='W-w':y='(H-h)/2':shortest=1,setsar=1,format=yuv420p[out]" \
    -map '[out]' \
    -t "$segment_duration" \
    -c:v libx264 -preset fast -crf 18 -video_track_timescale 24000 \
    "$output_path"
}

encode_portrait_photo() {
  local input_path="$1"
  local output_path="$2"
  local crop_top_fraction="${3:-0.0}"
  local crop_bottom_fraction="${4:-0.0}"
  local content_height="${5:-$portrait_height}"
  local crop_height_fraction

  crop_height_fraction=$(awk -v top="$crop_top_fraction" -v bottom="$crop_bottom_fraction" 'BEGIN { printf "%.10f", 1 - top - bottom }')

  ffmpeg -nostdin -hide_banner -loglevel error -y \
    -i "$input_path" \
    -an \
    -filter_complex "color=c=black:s=${frame_size}:r=${frame_rate}:d=${segment_duration}[bg-ready];[0:v]crop=iw:trunc(ih*${crop_height_fraction}/2)*2:0:trunc(ih*${crop_top_fraction}/2)*2,scale=${portrait_stage_width}:${portrait_stage_height}:force_original_aspect_ratio=increase:flags=lanczos,crop=${portrait_stage_width}:${portrait_stage_height}:(in_w-out_w)/2:(in_h-out_h)/2,unsharp=5:5:0.45:3:3:0,eq=contrast=1.04:saturation=1.04,tpad=stop_mode=clone:stop_duration=${segment_duration},fps=${frame_rate},setpts=PTS-STARTPTS,format=yuv444p,split=2[fg-color][fg-mask-source];[fg-mask-source]format=gray,geq=lum='clip(255*X/${edge_feather},0,255)'[fg-mask];[fg-color][fg-mask]alphamerge[fg-feather];[bg-ready][fg-feather]overlay=x='W-w':y='(H-h)/2':shortest=1,setsar=1,format=yuv420p[out]" \
    -map '[out]' \
    -t "$segment_duration" \
    -c:v libx264 -preset fast -crf 18 -video_track_timescale 24000 \
    "$output_path"
}

# The order moves from human-scale robotics through autonomy and inspection,
# then closes on fabrication and industrial process hardware.
encode_landscape_video "$source_dir/humanoid-robot-box-transfer-demo.mp4" "$work_dir/01.mp4" "3.200" "0.667" "0.50" "0.000" "$wide_stage_width"
encode_landscape_video "$source_dir/humanoid-teleoperation-news-clip.mp4" "$work_dir/03.mp4" "2.500" "0.667" "0.50"
encode_landscape_video "$source_dir/indoor-drone-flight-test.mov" "$work_dir/04.mp4" "2.500" "1.000" "0.50" "0.000" "$wide_stage_width"
encode_contained_video "$source_dir/turtlebot3-vision-guided-grasping-demo.mp4" "$work_dir/05.mp4" "4.100" "1.000" "0.00" "0.00"
encode_contained_video "$source_dir/ai-assisted-pcb-inspection-demo.mp4" "$work_dir/06.mp4" "12.900" "1.000" "0.00" "0.00" "6.000" "$frame_height"
encode_contained_video "$source_dir/robot-assisted-crack-detection-demo.mp4" "$work_dir/07.mp4" "6.500" "0.667" "0.00" "0.00"
encode_contained_video "$source_dir/3d-printer-fabrication-demo.mov" "$work_dir/08.mp4" "3.700" "0.667" "0.28" "0.12" "0.000" "$frame_height" "true"
encode_photo_card "$source_dir/automated-catalyst-cleaning-robot-components.jpg" "$work_dir/09.mp4" "0.30" "0.30"
encode_contained_video "$source_dir/automated-catalyst-cleaning-frame-grinding.mp4" "$work_dir/10.mp4" "2.500" "1.000" "0.00" "0.05" "5.200" "$portrait_height" "false" "2.500"
encode_contained_video "$source_dir/automated-catalyst-cleaning-robot-rail-test.mp4" "$work_dir/11.mp4" "2.500" "0.667" "0.30" "0.40" "0.000" "$frame_height" "true"
encode_contained_video "$source_dir/automated-mixing-system-agitation-test.mp4" "$work_dir/12.mp4" "2.500" "0.667" "0.30" "0.20" "0.000" "$frame_height" "true"

ffmpeg -nostdin -hide_banner -loglevel error -y \
  -i "$work_dir/01.mp4" -i "$work_dir/03.mp4" -i "$work_dir/04.mp4" \
  -i "$work_dir/05.mp4" -i "$work_dir/06.mp4" -i "$work_dir/07.mp4" \
  -i "$work_dir/08.mp4" -i "$work_dir/09.mp4" -i "$work_dir/10.mp4" \
  -i "$work_dir/11.mp4" -i "$work_dir/12.mp4" \
  -filter_complex "[0:v][1:v]xfade=transition=fade:duration=${transition_duration}:offset=2.825[x01];[x01][2:v]xfade=transition=fade:duration=${transition_duration}:offset=4.950[x02];[x02][3:v]xfade=transition=fade:duration=${transition_duration}:offset=7.075[x03];[x03][4:v]xfade=transition=fade:duration=${transition_duration}:offset=10.800[x04];[x04][5:v]xfade=transition=fade:duration=${transition_duration}:offset=23.325[x05];[x05][6:v]xfade=transition=fade:duration=${transition_duration}:offset=29.450[x06];[x06][7:v]xfade=transition=fade:duration=${transition_duration}:offset=32.775[x07];[x07][8:v]xfade=transition=fade:duration=${transition_duration}:offset=34.900[x08];[x08][9:v]xfade=transition=fade:duration=${transition_duration}:offset=37.025[x09];[x09][10:v]xfade=transition=fade:duration=${transition_duration}:offset=39.150,eq=contrast=1.04:saturation=0.88:brightness=-0.015,fade=t=in:st=0:d=0.35,fade=t=out:st=41.250:d=0.40,format=yuv420p[out]" \
  -map '[out]' -an \
  -c:v libx264 -preset slow -crf 23 -profile:v high -level 4.1 \
  -movflags +faststart \
  "$work_dir/robotics-automation-portfolio-reel.mp4"

ffmpeg -nostdin -hide_banner -loglevel error -y \
  -ss 0.8 -i "$work_dir/robotics-automation-portfolio-reel.mp4" \
  -frames:v 1 -vf "scale=${frame_width}:${frame_height}:flags=lanczos" \
  -c:v libwebp -quality 82 -update 1 \
  "$work_dir/robotics-automation-portfolio-reel-poster.webp"

mv -- "$work_dir/robotics-automation-portfolio-reel.mp4" "$video_dir/robotics-automation-portfolio-reel.mp4"
mv -- "$work_dir/robotics-automation-portfolio-reel-poster.webp" "$image_dir/robotics-automation-portfolio-reel-poster.webp"

printf 'Built %s\n' "$video_dir/robotics-automation-portfolio-reel.mp4"
printf 'Built %s\n' "$image_dir/robotics-automation-portfolio-reel-poster.webp"
